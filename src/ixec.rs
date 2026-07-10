//! Interleaved entropy coder — IPN 42-155 §IV.
//!
//! ICER does not use arithmetic coding for its entropy stage. §IV.A–D
//! specify a *bit-wise adaptable interleaved entropy coder*: a set of
//! component variable-to-variable-length binary source codes, one per
//! probability "bin", whose output codewords are interleaved into a
//! single stream so the decoder can reconstruct them in the same order
//! the encoder produced them.
//!
//! This module is the spec-exact realisation of §IV's component codes
//! and bin design. It is built bottom-up:
//!
//! * [`ComponentCode`] — one variable-to-variable-length code. A code is
//!   a bijective map between a *prefix-free + exhaustive* set of input
//!   codewords (bit strings consumed from the source) and an equally
//!   prefix-free + exhaustive set of output codewords (bit strings
//!   emitted to / consumed from the channel). §IV.B.
//! * The **Golomb codes** `G_m` (§IV.B): `m + 1` input codewords
//!   `1, 01, 001, …, 0^(m-1) 1, 0^m`, with the published output mapping.
//!   Used for bins 9–17 of Table 10.
//! * The **shorthand-tree codes** (§IV.D): bins 2–8 are specified by a
//!   decoding-tree shorthand string such as
//!   `(((((04 1, 14 ), 03 1), 001), 10), (01, (110, (05 , 13 0))))`.
//!   Each leaf is an input codeword; the path of branch labels from the
//!   root to a leaf is its output codeword (zero-branch listed first).
//! * The **bin design** ([`bins`]) — Table 10's 17 bins, each a
//!   probability cutoff `z_j` (denominator 65536) plus its component
//!   code. Bin 1 is the "uncoded" bin (each source bit is its own
//!   complete input *and* output codeword).
//!
//! * The **interleaving machinery** ([`InterleavedEncoder`] /
//!   [`InterleavedDecoder`], §IV.C) — the MER 2048-word circular buffer
//!   ([`BUFFER_WORDS`]), the FIFO front-of-list emission that keeps the
//!   channel in creation order, the §IV.C flush of partial words when the
//!   buffer fills or the input is exhausted, and the per-bin suffix
//!   bookkeeping the decoder uses to reverse it. Encoder and decoder both
//!   take the bin index per bit from the caller (in ICER that index comes
//!   from the §III.C probability estimate, reproduced identically on both
//!   sides).
//!
//! Wiring this coder in behind the context model — as an alternative to
//! the existing binary arithmetic coder — is the next milestone.

/// A variable-to-variable-length binary source code (IPN 42-155 §IV.B).
///
/// Both the input and output codeword sets are prefix-free and
/// exhaustive, so a bit stream can be uniquely parsed into codewords and
/// a complete codeword is recognised as soon as it is read. The code is
/// a bijection between the two sets:
///
/// * **encoding** parses a run of *source* bits into one input codeword,
///   then emits the paired output codeword;
/// * **decoding** parses a run of *channel* bits into one output
///   codeword, then emits the paired input codeword (§IV.B: "the same
///   procedure as encoding, with the roles of input and output codeword
///   sets reversed").
///
/// Codewords are stored MSB-first as `Vec<bool>` (a `true` is a `1`
/// branch / bit). Both sets are tabulated up front so encode and decode
/// are a pair of trie walks.
#[derive(Debug, Clone)]
pub struct ComponentCode {
    /// `(input_codeword, output_codeword)` pairs, in a stable order.
    pairs: Vec<(Vec<bool>, Vec<bool>)>,
}

/// Result of consuming bits through a [`Trie`]: either a codeword index
/// was completed (and how many bits it consumed), or more bits are
/// needed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum TrieStep {
    /// A complete codeword `index` was recognised after `consumed` bits.
    Complete { index: usize, consumed: usize },
    /// The bits seen so far are a strict prefix of one or more codewords;
    /// feed more.
    More,
    /// The bits seen so far match no codeword prefix (malformed input).
    Dead,
}

/// A binary trie over a prefix-free codeword set, mapping a recognised
/// codeword to its position (the codeword's index in the set).
#[derive(Debug, Clone)]
struct Trie {
    /// Node 0 is the root. Each node carries optional `(zero_child,
    /// one_child)` links and, at a leaf, the codeword index it terminates.
    nodes: Vec<TrieNode>,
}

#[derive(Debug, Clone, Default)]
struct TrieNode {
    zero: Option<usize>,
    one: Option<usize>,
    /// `Some(idx)` if this node terminates codeword `idx`.
    terminal: Option<usize>,
}

impl Trie {
    fn build(codewords: &[Vec<bool>]) -> Self {
        let mut nodes = vec![TrieNode::default()];
        for (idx, cw) in codewords.iter().enumerate() {
            let mut cur = 0usize;
            for &bit in cw {
                let next = if bit { nodes[cur].one } else { nodes[cur].zero };
                cur = match next {
                    Some(n) => n,
                    None => {
                        let n = nodes.len();
                        nodes.push(TrieNode::default());
                        if bit {
                            nodes[cur].one = Some(n);
                        } else {
                            nodes[cur].zero = Some(n);
                        }
                        n
                    }
                };
            }
            nodes[cur].terminal = Some(idx);
        }
        Trie { nodes }
    }

    /// Walk `bits` from the root, returning whether a codeword completed.
    fn walk(&self, bits: &[bool]) -> TrieStep {
        let mut cur = 0usize;
        for (i, &bit) in bits.iter().enumerate() {
            if let Some(idx) = self.nodes[cur].terminal {
                // Prefix-free sets never have a terminal with children,
                // so this only fires if a full codeword was already read.
                return TrieStep::Complete {
                    index: idx,
                    consumed: i,
                };
            }
            let next = if bit {
                self.nodes[cur].one
            } else {
                self.nodes[cur].zero
            };
            match next {
                Some(n) => cur = n,
                None => return TrieStep::Dead,
            }
        }
        if let Some(idx) = self.nodes[cur].terminal {
            TrieStep::Complete {
                index: idx,
                consumed: bits.len(),
            }
        } else {
            TrieStep::More
        }
    }
}

impl ComponentCode {
    /// Build a component code from explicit `(input, output)` codeword
    /// pairs. Both sides must be prefix-free + exhaustive (the caller's
    /// responsibility; the constructors below produce valid sets).
    fn from_pairs(pairs: Vec<(Vec<bool>, Vec<bool>)>) -> Self {
        ComponentCode { pairs }
    }

    /// Number of (input, output) codeword pairs.
    pub fn len(&self) -> usize {
        self.pairs.len()
    }

    /// Whether the code has no codewords (never true for a valid code).
    pub fn is_empty(&self) -> bool {
        self.pairs.is_empty()
    }

    /// The §IV.B Golomb code `G_m` (`m >= 1`).
    ///
    /// Input codewords: `1, 01, 001, …, 0^(m-1) 1, 0^m` — i.e. for
    /// `k = 0..m` the word `0^k 1`, plus the all-zeros word `0^m`.
    ///
    /// Output codewords (§IV.B): input `0^m` maps to the single bit `1`.
    /// For input `0^k 1` (`k < m`), let `ℓ = ⌈log2 m⌉` and `i = 2^ℓ − m`;
    /// the output is the `ℓ`-bit binary of `k` when `k < i`, else the
    /// `(ℓ+1)`-bit binary of `k + i`.
    pub fn golomb(m: u32) -> Self {
        assert!(m >= 1, "Golomb parameter m must be >= 1");
        let l = ceil_log2(m);
        let i = (1u32 << l) - m;
        let mut pairs: Vec<(Vec<bool>, Vec<bool>)> = Vec::with_capacity(m as usize + 1);
        for k in 0..m {
            // Input codeword 0^k 1.
            let mut input = vec![false; k as usize];
            input.push(true);
            // Output codeword.
            let output = if k < i {
                int_to_bits(k, l)
            } else {
                int_to_bits(k + i, l + 1)
            };
            pairs.push((input, output));
        }
        // Input codeword 0^m -> output single 1.
        pairs.push((vec![false; m as usize], vec![true]));
        ComponentCode::from_pairs(pairs)
    }

    /// Build a component code from the §IV.D shorthand decoding-tree
    /// notation. Each leaf of the tree is an *input* codeword (written in
    /// the `0^a 1^b …` run shorthand); the path of branch labels from the
    /// root to that leaf — zero-branch first in each `(left, right)` pair
    /// — is the matching *output* codeword.
    pub fn from_shorthand(spec: &str) -> Self {
        let tokens = tokenize_shorthand(spec);
        let mut pos = 0usize;
        let mut pairs = Vec::new();
        let mut prefix = Vec::new();
        parse_tree(&tokens, &mut pos, &mut prefix, &mut pairs);
        debug_assert_eq!(pos, tokens.len(), "shorthand fully consumed");
        ComponentCode::from_pairs(pairs)
    }

    /// The "uncoded" bin-1 code: each single source bit is its own
    /// complete input codeword and maps to the identical output bit
    /// (§IV.D: bin 1 bits "are unchanged by the coding process").
    pub fn uncoded() -> Self {
        ComponentCode::from_pairs(vec![(vec![false], vec![false]), (vec![true], vec![true])])
    }

    /// Encode a run of source bits: parse exactly one *input* codeword
    /// from the front of `bits`, returning the matched output codeword
    /// and the number of source bits consumed. Returns `None` if `bits`
    /// is only a (strict) prefix of an input codeword (caller must
    /// supply more) — a malformed dead-end also yields `None`.
    pub fn encode_one(&self, bits: &[bool]) -> Option<(Vec<bool>, usize)> {
        let inputs: Vec<Vec<bool>> = self.pairs.iter().map(|(i, _)| i.clone()).collect();
        let trie = Trie::build(&inputs);
        match trie.walk(bits) {
            TrieStep::Complete { index, consumed } => Some((self.pairs[index].1.clone(), consumed)),
            _ => None,
        }
    }

    /// Decode: parse exactly one *output* codeword from the front of
    /// `bits` (channel bits), returning the matched input codeword and
    /// the number of channel bits consumed. `None` on incomplete / dead.
    pub fn decode_one(&self, bits: &[bool]) -> Option<(Vec<bool>, usize)> {
        let outputs: Vec<Vec<bool>> = self.pairs.iter().map(|(_, o)| o.clone()).collect();
        let trie = Trie::build(&outputs);
        match trie.walk(bits) {
            TrieStep::Complete { index, consumed } => Some((self.pairs[index].0.clone(), consumed)),
            _ => None,
        }
    }

    /// The longest input codeword length, in bits — the maximum number
    /// of source bits a single `encode_one` can consume.
    pub fn max_input_len(&self) -> usize {
        self.pairs.iter().map(|(i, _)| i.len()).max().unwrap_or(0)
    }

    /// Look up the output codeword paired with a *complete* input
    /// codeword `input`, or `None` if `input` is not a codeword.
    fn output_for_input(&self, input: &[bool]) -> Option<&[bool]> {
        self.pairs
            .iter()
            .find(|(i, _)| i.as_slice() == input)
            .map(|(_, o)| o.as_slice())
    }

    /// Look up the input codeword paired with a *complete* output
    /// codeword `output`, or `None`.
    fn input_for_output(&self, output: &[bool]) -> Option<&[bool]> {
        self.pairs
            .iter()
            .find(|(_, o)| o.as_slice() == output)
            .map(|(i, _)| i.as_slice())
    }

    /// Classify a run of source bits against the *input* codeword set:
    /// is `bits` exactly a codeword, a strict prefix of one or more, or
    /// neither?
    fn classify_input(&self, bits: &[bool]) -> InputStatus {
        // Exact match?
        if self.pairs.iter().any(|(i, _)| i.as_slice() == bits) {
            return InputStatus::Complete;
        }
        // Strict prefix of some codeword?
        if self
            .pairs
            .iter()
            .any(|(i, _)| i.len() > bits.len() && i[..bits.len()] == *bits)
        {
            InputStatus::Partial
        } else {
            InputStatus::None
        }
    }

    /// The §IV.C flush of a *partial* input prefix: the shortest input
    /// codeword that has `prefix` as a prefix (ties broken by the order
    /// the codewords appear). Returns that completed input codeword and
    /// its paired output codeword. §IV.C: "the shortest output codeword
    /// consistent with the bits already in the partial codeword".
    ///
    /// An empty `prefix` (no bits accumulated) has no codeword to flush;
    /// callers only flush non-empty partials.
    fn flush_partial(&self, prefix: &[bool]) -> Option<(Vec<bool>, Vec<bool>)> {
        self.pairs
            .iter()
            .filter(|(i, _)| i.len() >= prefix.len() && i[..prefix.len()] == *prefix)
            .min_by_key(|(_, o)| o.len())
            .map(|(i, o)| (i.clone(), o.clone()))
    }

    /// The set of `(input, output)` pairs (for tests / inspection).
    #[cfg(test)]
    pub(crate) fn pairs(&self) -> &[(Vec<bool>, Vec<bool>)] {
        &self.pairs
    }
}

/// `⌈log2 m⌉` for `m >= 1`.
fn ceil_log2(m: u32) -> u32 {
    debug_assert!(m >= 1);
    if m == 1 {
        return 0;
    }
    32 - (m - 1).leading_zeros()
}

/// The `width`-bit big-endian binary representation of `value` as a
/// MSB-first bool vector.
fn int_to_bits(value: u32, width: u32) -> Vec<bool> {
    (0..width).rev().map(|b| (value >> b) & 1 == 1).collect()
}

/// One token of the §IV.D shorthand grammar.
#[derive(Debug, Clone, PartialEq, Eq)]
enum Token {
    Open,
    Close,
    Comma,
    /// A leaf codeword expressed as its literal bit string.
    Leaf(Vec<bool>),
}

/// Tokenize a §IV.D shorthand string. Leaf codewords use the run
/// shorthand: the paper writes runs as a superscript exponent (`0^i`
/// denotes `i` zeros). This transcription marks a run with an explicit
/// caret: `0^4 1` means `0000` followed by `1` = `00001`; `1^3 0` means
/// `111` followed by `0`. A bare `0` / `1` (no caret) is a single bit,
/// so `01` is the two-bit string zero-one — *not* a run. Whitespace and
/// the carets are flattened into the literal bit vector.
fn tokenize_shorthand(spec: &str) -> Vec<Token> {
    let chars: Vec<char> = spec.chars().collect();
    let mut tokens = Vec::new();
    let mut i = 0usize;
    while i < chars.len() {
        match chars[i] {
            '(' => {
                tokens.push(Token::Open);
                i += 1;
            }
            ')' => {
                tokens.push(Token::Close);
                i += 1;
            }
            ',' => {
                tokens.push(Token::Comma);
                i += 1;
            }
            c if c.is_whitespace() => {
                i += 1;
            }
            '0' | '1' => {
                // Accumulate a maximal run of bit/run tokens into one
                // leaf, stopping at a comma / paren / EOF.
                let mut bits = Vec::new();
                while i < chars.len() {
                    match chars[i] {
                        '0' | '1' => {
                            let bit = chars[i] == '1';
                            i += 1;
                            // A caret introduces an explicit run length
                            // (the paper's superscript exponent); without
                            // it the bit is a single literal bit.
                            let mut count = 1usize;
                            if i < chars.len() && chars[i] == '^' {
                                i += 1;
                                let mut digits = String::new();
                                while i < chars.len() && chars[i].is_ascii_digit() {
                                    digits.push(chars[i]);
                                    i += 1;
                                }
                                count = digits.parse().expect("run length after caret");
                            }
                            for _ in 0..count {
                                bits.push(bit);
                            }
                        }
                        c if c.is_whitespace() => {
                            i += 1;
                        }
                        _ => break,
                    }
                }
                tokens.push(Token::Leaf(bits));
            }
            other => panic!("unexpected char {other:?} in shorthand"),
        }
    }
    tokens
}

/// Recursively parse a shorthand subtree starting at `tokens[*pos]`,
/// accumulating leaf `(input, output)` pairs into `pairs`. `prefix` is
/// the output-bit path from the root to the current node.
fn parse_tree(
    tokens: &[Token],
    pos: &mut usize,
    prefix: &mut Vec<bool>,
    pairs: &mut Vec<(Vec<bool>, Vec<bool>)>,
) {
    match &tokens[*pos] {
        Token::Leaf(bits) => {
            pairs.push((bits.clone(), prefix.clone()));
            *pos += 1;
        }
        Token::Open => {
            *pos += 1; // consume '('
                       // Left child: zero branch.
            prefix.push(false);
            parse_tree(tokens, pos, prefix, pairs);
            prefix.pop();
            // Expect a comma.
            debug_assert_eq!(tokens[*pos], Token::Comma, "expected comma in subtree");
            *pos += 1;
            // Right child: one branch.
            prefix.push(true);
            parse_tree(tokens, pos, prefix, pairs);
            prefix.pop();
            // Expect a close paren.
            debug_assert_eq!(tokens[*pos], Token::Close, "expected close paren");
            *pos += 1;
        }
        other => panic!("unexpected token {other:?} parsing shorthand tree"),
    }
}

/// One bin of ICER's interleaved entropy coder (IPN 42-155 §IV.D
/// Table 10).
#[derive(Debug, Clone)]
pub struct Bin {
    /// 1-based bin index `j` (Table 10 column 1).
    pub index: u8,
    /// Probability cutoff `z_j` numerator over the fixed denominator
    /// [`Z_DENOM`] = 65536. Bin `j` covers probability-of-zero interval
    /// `[z_{j-1}, z_j)` with `z_0 = 1/2`.
    pub cutoff_num: u32,
    /// The bin's component code.
    pub code: ComponentCode,
}

/// The fixed denominator for Table 10 probability cutoffs (§IV.D: "we
/// use cutoffs that are rational numbers with denominator 2^16").
pub const Z_DENOM: u32 = 65536;

/// `z_0 = 1/2` as a numerator over [`Z_DENOM`].
pub const Z0_NUM: u32 = Z_DENOM / 2;

/// The Table 10 cutoff numerators `z_1 .. z_17` over [`Z_DENOM`].
///
/// Bin `j` (1-based) has cutoff `CUTOFFS[j-1]`; its interval is
/// `[z_{j-1}, z_j)` with `z_0 = 32768/65536 = 1/2`.
pub const CUTOFFS: [u32; 17] = [
    35298, 37345, 40503, 43591, 47480, 50133, 53645, 55902, 57755, 58894, 60437, 62267, 63613,
    64557, 65134, 65392, 65536,
];

/// Build the 17-bin component-code design of ICER's interleaved entropy
/// coder (IPN 42-155 §IV.D Table 10).
///
/// * Bin 1 — the "uncoded" bin: each source bit passes through
///   unchanged.
/// * Bins 2–8 — the §IV.D shorthand-tree component codes.
/// * Bins 9–17 — the Golomb codes G5, G6, G7, G11, G17, G31, G70, G200,
///   G512.
pub fn bins() -> Vec<Bin> {
    // §IV.D shorthand strings for bins 2..=8, transcribed from Table 10.
    let sh2 = "(((((0^4 1, 1^4 ), 0^3 1), 001), 10), (01, (110, (0^5 , 1^3 0))))";
    let sh3 = "(((001, ((1101, 0^3 11), 1^3 )), 10), (01, (0^4 , (1100, 0^3 10))))";
    let sh4 = "((0^3 , 01), (10, (001, 11)))";
    let sh5 = "(((010, (10^4 , 110)), ((101, 011), ((10^3 1, 1^3 ), 1001))), 00)";
    let sh6 = "((0^5 , 1), ((0^3 1, 001), (010, (0^4 1, 011))))";
    let sh7 = "(0^3 , ((001, 010), (100, (11, (011, 101)))))";
    let sh8 = "(0^4 , ((001, 01), (10, (0^3 10, (0^3 11, 11)))))";

    let codes = [
        ComponentCode::uncoded(),
        ComponentCode::from_shorthand(sh2),
        ComponentCode::from_shorthand(sh3),
        ComponentCode::from_shorthand(sh4),
        ComponentCode::from_shorthand(sh5),
        ComponentCode::from_shorthand(sh6),
        ComponentCode::from_shorthand(sh7),
        ComponentCode::from_shorthand(sh8),
        ComponentCode::golomb(5),
        ComponentCode::golomb(6),
        ComponentCode::golomb(7),
        ComponentCode::golomb(11),
        ComponentCode::golomb(17),
        ComponentCode::golomb(31),
        ComponentCode::golomb(70),
        ComponentCode::golomb(200),
        ComponentCode::golomb(512),
    ];

    codes
        .into_iter()
        .enumerate()
        .map(|(idx, code)| Bin {
            index: (idx + 1) as u8,
            cutoff_num: CUTOFFS[idx],
            code,
        })
        .collect()
}

/// Select the Table 10 bin index (1-based) for a probability-of-zero
/// estimate `p = p_num / p_den`, after the §IV.C `p >= 1/2` reduction.
///
/// §IV.C: "Without loss of generality, we may assume that `p_i >= 1/2`."
/// A probability below 1/2 is folded above 1/2 by the caller (inverting
/// the source bit); this function assumes `p >= 1/2` and locates the bin
/// `j` whose interval `[z_{j-1}, z_j)` contains `p`. Bin `j` is the
/// smallest index with `p < z_j` (and `p >= z_{j-1}`); a `p` at or above
/// the final cutoff lands in bin 17.
pub fn bin_for_probability(p_num: u32, p_den: u32) -> u8 {
    // Compare p = p_num/p_den against each cutoff z_j = CUTOFFS[j-1]/2^16
    // using cross-multiplication to stay in integer arithmetic:
    // p < z_j  <=>  p_num * Z_DENOM < CUTOFFS[j-1] * p_den.
    let lhs = p_num as u64 * Z_DENOM as u64;
    for (idx, &cut) in CUTOFFS.iter().enumerate() {
        let rhs = cut as u64 * p_den as u64;
        if lhs < rhs {
            return (idx + 1) as u8;
        }
    }
    17
}

/// Classification of a run of source bits against a component code's
/// input-codeword set.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum InputStatus {
    /// `bits` is exactly an input codeword.
    Complete,
    /// `bits` is a strict prefix of one or more input codewords.
    Partial,
    /// `bits` is neither a codeword nor a prefix (cannot happen for an
    /// exhaustive prefix-free set fed valid bits).
    None,
}

/// The MER circular-buffer capacity, in words (§IV.C.1: "the circular
/// buffer has a capacity of 2048 words").
pub const BUFFER_WORDS: usize = 2048;

/// One in-flight word in the encoder's list: the bin it belongs to plus
/// the input-codeword bits accumulated so far.
#[derive(Debug, Clone)]
struct Word {
    bin: usize,
    bits: Vec<bool>,
    /// Set once `bits` is a complete input codeword for `bin`'s code.
    complete: bool,
}

/// The interleaved entropy encoder (IPN 42-155 §IV.C.1 + §IV.D).
///
/// Source bits arrive one at a time, each with the index (1-based, as in
/// Table 10) of the bin its probability estimate selected. The encoder
/// groups bits of the same bin into input codewords, holds the
/// partially- and fully-formed words in an ordered list (the MER
/// circular buffer of [`BUFFER_WORDS`] words), and emits each word's
/// output codeword when that word reaches the *front* of the list
/// complete — preserving the order the decoder needs.
pub struct InterleavedEncoder {
    bins: Vec<Bin>,
    /// Ordered word list (front = index 0). At most one *partial* word
    /// per bin is open at a time (the most recent).
    words: std::collections::VecDeque<Word>,
    /// The emitted channel bits.
    out: BitSink,
    /// Number of mid-stream buffer-full front flushes performed
    /// (§IV.C.1). Test-only observability: the flush-desync regression
    /// tests assert their streams genuinely exercise this path.
    #[cfg(test)]
    pub(crate) mid_stream_flushes: usize,
}

/// A growable MSB-first bit buffer that packs into bytes on `finish`.
#[derive(Debug, Default)]
struct BitSink {
    bits: Vec<bool>,
}

impl BitSink {
    fn push_bits(&mut self, bits: &[bool]) {
        self.bits.extend_from_slice(bits);
    }

    /// Pack the accumulated bits MSB-first into bytes (zero-padding the
    /// final partial byte). The interleaved decoder reads bits back in
    /// the same MSB-first order and stops once every source bit is
    /// recovered, so trailing pad bits are harmless.
    fn into_bytes(self) -> Vec<u8> {
        let mut out = Vec::with_capacity(self.bits.len().div_ceil(8));
        let mut cur = 0u8;
        let mut n = 0u8;
        for b in self.bits {
            cur = (cur << 1) | (b as u8);
            n += 1;
            if n == 8 {
                out.push(cur);
                cur = 0;
                n = 0;
            }
        }
        if n != 0 {
            out.push(cur << (8 - n));
        }
        out
    }
}

impl InterleavedEncoder {
    /// Build a fresh encoder over the Table 10 bin design.
    pub fn new() -> Self {
        InterleavedEncoder {
            bins: bins(),
            words: std::collections::VecDeque::new(),
            out: BitSink::default(),
            #[cfg(test)]
            mid_stream_flushes: 0,
        }
    }

    /// Encode one source bit assigned to bin `bin_index` (1-based).
    ///
    /// The bit is appended to the bin's open partial word (or starts a
    /// new word at the tail of the list). If that word becomes a complete
    /// input codeword it is flagged; then any complete words at the
    /// *front* of the list are drained, emitting their output codewords.
    /// A full buffer triggers a §IV.C flush of the front word.
    pub fn encode_bit(&mut self, bit: bool, bin_index: u8) {
        let bin = bin_index as usize - 1;
        debug_assert!(bin < self.bins.len());

        // Find this bin's open partial word (scanning from the back: the
        // most-recent open word for the bin). A word stays "open" until
        // it is complete.
        let open = self
            .words
            .iter_mut()
            .rev()
            .find(|w| w.bin == bin && !w.complete);

        match open {
            Some(w) => {
                w.bits.push(bit);
                let status = self.bins[bin].code.classify_input(&w.bits);
                if status == InputStatus::Complete {
                    w.complete = true;
                }
            }
            None => {
                let mut bits = Vec::with_capacity(4);
                bits.push(bit);
                let complete = self.bins[bin].code.classify_input(&bits) == InputStatus::Complete;
                self.words.push_back(Word {
                    bin,
                    bits,
                    complete,
                });
            }
        }

        self.drain_front();

        // §IV.C.1: if the buffer is full, flush the front (partial) word.
        while self.words.len() >= BUFFER_WORDS {
            #[cfg(test)]
            {
                self.mid_stream_flushes += 1;
            }
            self.flush_front();
            self.drain_front();
        }
    }

    /// Emit + remove every complete word at the front of the list.
    fn drain_front(&mut self) {
        while let Some(front) = self.words.front() {
            if !front.complete {
                break;
            }
            let front = self.words.pop_front().unwrap();
            let out = self.bins[front.bin]
                .code
                .output_for_input(&front.bits)
                .expect("complete word has an output codeword");
            self.out.push_bits(out);
        }
    }

    /// §IV.C.1 flush of the front word: complete its partial input
    /// codeword with the shortest consistent output codeword, then drain.
    fn flush_front(&mut self) {
        let Some(front) = self.words.front().cloned() else {
            return;
        };
        // The front is necessarily partial here (drain_front already
        // removed any complete front). Flush it to a full codeword.
        if let Some((full_input, _)) = self.bins[front.bin].code.flush_partial(&front.bits) {
            if let Some(w) = self.words.front_mut() {
                w.bits = full_input;
                w.complete = true;
            }
        } else {
            // No codeword extends the prefix (cannot happen for a valid
            // exhaustive code); drop the word to guarantee progress.
            self.words.pop_front();
        }
    }

    /// Finish encoding: flush all remaining partial words (§IV.C.1
    /// "flush bits also are used to complete all partial codewords
    /// remaining in the list once the input bit sequence is exhausted"),
    /// then pack to bytes. Returns the channel byte stream.
    pub fn finish(mut self) -> Vec<u8> {
        // Complete every word still in the list, front to back. A word
        // that is already complete emits directly; a partial one is
        // flushed to its shortest consistent codeword first. Mirrors the
        // mid-stream FIFO drain so the channel emission order stays
        // creation order (the order the decoder reconstructs in).
        while !self.words.is_empty() {
            let len_before = self.words.len();
            if !self.words.front().unwrap().complete {
                self.flush_front();
            }
            self.drain_front();
            // Guard against a stuck front (only possible on a malformed
            // code where `flush_partial` found no completion): if no word
            // was removed this iteration, drop the front to guarantee
            // termination.
            if self.words.len() == len_before {
                self.words.pop_front();
            }
        }
        self.out.into_bytes()
    }
}

impl Default for InterleavedEncoder {
    fn default() -> Self {
        Self::new()
    }
}

/// The interleaved entropy decoder (IPN 42-155 §IV.C.2 + §IV.D).
///
/// The decoder keeps, per bin, a *suffix* of the next input codeword for
/// that bin. To decode a source bit assigned to bin `j` it either pops
/// the first bit of that bin's stored suffix, or — if the suffix is
/// empty — parses one output codeword from the channel stream, maps it
/// back to its input codeword, returns the input codeword's first bit,
/// and stores the remainder as the new suffix.
pub struct InterleavedDecoder<'a> {
    bins: Vec<Bin>,
    /// Channel bits (MSB-first) with a read cursor.
    chan: &'a [u8],
    bit_pos: usize,
    /// Per-bin stored suffix of the in-progress input codeword.
    suffix: Vec<std::collections::VecDeque<bool>>,
    /// Mirror of the **encoder's** word list (§IV.C.1), rebuilt from the
    /// decoded source bits. The decoder learns every source bit in
    /// arrival order, so it can replay the encoder's list state exactly
    /// — which is what lets it *detect the mid-stream buffer-full
    /// flush*: when the mirrored list reaches [`BUFFER_WORDS`], the
    /// encoder flushed its front partial word by completing it with
    /// flush bits the source never produced. Those flush bits are
    /// exactly the not-yet-consumed remainder of that bin's parsed
    /// input codeword (every genuinely-arrived bit of the flushed word
    /// was already served — source bits are consumed in order), so the
    /// decoder discards the bin's suffix and stays in sync with the
    /// fresh word the encoder opens for that bin's next source bit.
    words: std::collections::VecDeque<Word>,
}

impl<'a> InterleavedDecoder<'a> {
    /// Build a decoder over the channel byte stream produced by
    /// [`InterleavedEncoder::finish`].
    pub fn new(channel: &'a [u8]) -> Self {
        let bins = bins();
        let n = bins.len();
        InterleavedDecoder {
            bins,
            chan: channel,
            bit_pos: 0,
            suffix: (0..n).map(|_| std::collections::VecDeque::new()).collect(),
            words: std::collections::VecDeque::new(),
        }
    }

    /// Read the next channel bit (MSB-first), or `false` past the end
    /// (trailing reads after the packed stream are pad bits — the caller
    /// only decodes as many source bits as were encoded).
    fn read_chan_bit(&mut self) -> bool {
        let byte = self.bit_pos / 8;
        if byte >= self.chan.len() {
            return false;
        }
        let bit = (self.chan[byte] >> (7 - (self.bit_pos % 8))) & 1 == 1;
        self.bit_pos += 1;
        bit
    }

    /// Decode one source bit assigned to bin `bin_index` (1-based).
    pub fn decode_bit(&mut self, bin_index: u8) -> bool {
        let bin = bin_index as usize - 1;
        if self.suffix[bin].is_empty() {
            // Reconstruct one input codeword by parsing an output
            // codeword from the channel (§IV.C.2).
            let input = self.parse_input_codeword(bin);
            for b in input {
                self.suffix[bin].push_back(b);
            }
        }
        // The first remaining bit of the suffix is the decoded bit.
        let bit = self.suffix[bin].pop_front().unwrap_or(false);
        self.replay_encoder_list(bit, bin);
        bit
    }

    /// Replay the encoder's §IV.C.1 word-list transition for one decoded
    /// source bit (see the [`Self::words`] field docs): append the bit
    /// to the bin's open word (or open a new word), drain complete front
    /// words, and — when the mirrored list reaches [`BUFFER_WORDS`] —
    /// reproduce the encoder's mid-stream flush by discarding the
    /// flushed bin's pending suffix (the flush bits).
    fn replay_encoder_list(&mut self, bit: bool, bin: usize) {
        let open = self
            .words
            .iter_mut()
            .rev()
            .find(|w| w.bin == bin && !w.complete);
        match open {
            Some(w) => {
                w.bits.push(bit);
                if self.bins[bin].code.classify_input(&w.bits) == InputStatus::Complete {
                    w.complete = true;
                }
            }
            None => {
                let bits = vec![bit];
                let complete = self.bins[bin].code.classify_input(&bits) == InputStatus::Complete;
                self.words.push_back(Word {
                    bin,
                    bits,
                    complete,
                });
            }
        }
        // Mirror of the encoder's drain_front: complete front words left
        // the encoder's list (their codewords were emitted).
        while self.words.front().is_some_and(|w| w.complete) {
            self.words.pop_front();
        }
        // Mirror of the encoder's buffer-full flush: the front partial
        // word was completed with flush bits and emitted. Every arrived
        // bit of that word has already been decoded (source bits are
        // consumed in order), so the bin's remaining suffix holds
        // exactly the flush bits — drop them.
        while self.words.len() >= BUFFER_WORDS {
            if let Some(front) = self.words.pop_front() {
                self.suffix[front.bin].clear();
            }
            while self.words.front().is_some_and(|w| w.complete) {
                self.words.pop_front();
            }
        }
    }

    /// Parse one output codeword from the channel and return its paired
    /// input codeword (§IV.C.2). Reads channel bits until the output
    /// trie recognises a complete output codeword.
    fn parse_input_codeword(&mut self, bin: usize) -> Vec<bool> {
        let mut acc: Vec<bool> = Vec::with_capacity(4);
        loop {
            acc.push(self.read_chan_bit());
            if let Some(input) = self.bins[bin].code.input_for_output(&acc) {
                return input.to_vec();
            }
            // Safety bound: no output codeword in Table 10 exceeds a
            // small length; a runaway means the channel is exhausted /
            // corrupt, so stop.
            if acc.len() > 64 {
                return vec![false];
            }
        }
    }
}

/// Map a probability-of-*one* estimate `(p1_num, p1_den)` — the form the
/// §III.C context model produces and the arithmetic coder consumes — to
/// the §IV bin index plus a "bit was inverted" flag.
///
/// §IV.C requires the coded bit's probability-of-zero `p0 = 1 - p1` to be
/// `>= 1/2`. When `p0 < 1/2` (i.e. `p1 > 1/2`) the source bit is inverted
/// before coding so the bin always sees `p0 >= 1/2`; the same inversion
/// is reproducible in the decoder. Returns `(bin_index, invert)` where
/// `invert` is whether the coded bit is the *complement* of the source
/// bit.
fn bin_and_invert(p1_num: u32, p1_den: u32) -> (u8, bool) {
    // Guard a degenerate denominator.
    let den = p1_den.max(1);
    let p1 = p1_num.min(den);
    let p0 = den - p1; // probability-of-zero numerator over `den`.
    if 2 * p0 >= den {
        // p0 >= 1/2: code the bit as-is, p = p0.
        (bin_for_probability(p0, den), false)
    } else {
        // p0 < 1/2: invert so the coded bit has probability-of-zero p1.
        (bin_for_probability(p1, den), true)
    }
}

/// Context-driven interleaved entropy *encoder* — a drop-in for
/// [`crate::arith::ArithEncoder`] that codes via ICER's §IV interleaved
/// coder instead of binary arithmetic coding.
///
/// `encode_bit(symbol, p1_num, p1_den)` takes the same probability-of-one
/// estimate the arithmetic coder does. Internally it applies the §IV.C
/// `p0 >= 1/2` reduction, selects the Table 10 bin, and feeds the
/// (possibly inverted) bit to the [`InterleavedEncoder`].
pub struct IxecEncoder {
    inner: InterleavedEncoder,
}

impl IxecEncoder {
    pub fn new() -> Self {
        IxecEncoder {
            inner: InterleavedEncoder::new(),
        }
    }

    /// Encode one binary `symbol` (0 or 1) whose probability of being a
    /// `1` is `p1_num / p1_den`.
    pub fn encode_bit(&mut self, symbol: u8, p1_num: u32, p1_den: u32) {
        let (bin, invert) = bin_and_invert(p1_num, p1_den);
        let coded = if invert { symbol == 0 } else { symbol == 1 };
        // The interleaver codes a bool where the bin's probability-of-zero
        // applies to the `false` value, so the "zero" of the coded bit is
        // `false`. We pass the coded source bit directly: `true` is a one.
        self.inner.encode_bit(coded, bin);
    }

    /// Flush + return the channel byte stream.
    pub fn finish(self) -> Vec<u8> {
        self.inner.finish()
    }
}

impl Default for IxecEncoder {
    fn default() -> Self {
        Self::new()
    }
}

/// Context-driven interleaved entropy *decoder* — the inverse of
/// [`IxecEncoder`], a drop-in for [`crate::arith::ArithDecoder`].
pub struct IxecDecoder<'a> {
    inner: InterleavedDecoder<'a>,
}

impl<'a> IxecDecoder<'a> {
    pub fn new(channel: &'a [u8]) -> Self {
        IxecDecoder {
            inner: InterleavedDecoder::new(channel),
        }
    }

    /// Decode one binary symbol given the same probability-of-one
    /// estimate the encoder used.
    pub fn decode_bit(&mut self, p1_num: u32, p1_den: u32) -> u8 {
        let (bin, invert) = bin_and_invert(p1_num, p1_den);
        let coded = self.inner.decode_bit(bin);
        // Reverse the §IV.C inversion to recover the source bit.
        let symbol = if invert { !coded } else { coded };
        symbol as u8
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn bits(s: &str) -> Vec<bool> {
        s.chars().map(|c| c == '1').collect()
    }

    #[test]
    fn ceil_log2_basic() {
        assert_eq!(ceil_log2(1), 0);
        assert_eq!(ceil_log2(2), 1);
        assert_eq!(ceil_log2(3), 2);
        assert_eq!(ceil_log2(4), 2);
        assert_eq!(ceil_log2(5), 3);
        assert_eq!(ceil_log2(8), 3);
        assert_eq!(ceil_log2(9), 4);
        assert_eq!(ceil_log2(512), 9);
    }

    #[test]
    fn int_to_bits_msb_first() {
        assert_eq!(int_to_bits(0, 3), bits("000"));
        assert_eq!(int_to_bits(6, 4), bits("0110"));
        assert_eq!(int_to_bits(7, 4), bits("0111"));
        assert_eq!(int_to_bits(2, 3), bits("010"));
    }

    /// IPN 42-155 §IV.B Table 9 — the Golomb code G5. Input/output
    /// codeword pairs must match the published table exactly.
    #[test]
    fn golomb_g5_matches_table9() {
        let g5 = ComponentCode::golomb(5);
        // Table 9 (input -> output), input MSB-first:
        //   00000 -> 1
        //   00001 -> 0111
        //   0001  -> 0110
        //   001   -> 010
        //   01    -> 001
        //   1     -> 000
        let expect: Vec<(&str, &str)> = vec![
            ("1", "000"),
            ("01", "001"),
            ("001", "010"),
            ("0001", "0110"),
            ("00001", "0111"),
            ("00000", "1"),
        ];
        let pairs = g5.pairs();
        assert_eq!(pairs.len(), 6, "G5 has m+1 = 6 codewords");
        for (inp, outp) in expect {
            let found = pairs
                .iter()
                .find(|(i, _)| *i == bits(inp))
                .unwrap_or_else(|| panic!("G5 missing input {inp}"));
            assert_eq!(found.1, bits(outp), "G5 output for input {inp}");
        }
    }

    /// Both codeword sets of every Golomb code are prefix-free: building
    /// a trie and walking each codeword recognises exactly itself.
    #[test]
    fn golomb_codeword_sets_prefix_free_and_bijective() {
        for &m in &[1u32, 2, 5, 6, 7, 11, 17, 31, 70, 200, 512] {
            let g = ComponentCode::golomb(m);
            assert_eq!(g.len(), m as usize + 1, "G{m} has m+1 codewords");
            // Encode each input codeword, decode the output, recover input.
            for (input, output) in g.pairs() {
                let (enc_out, used_in) = g.encode_one(input).expect("encode_one");
                assert_eq!(&enc_out, output, "G{m} encode input {input:?}");
                assert_eq!(used_in, input.len(), "G{m} consumed full input");
                let (dec_in, used_out) = g.decode_one(output).expect("decode_one");
                assert_eq!(&dec_in, input, "G{m} decode output {output:?}");
                assert_eq!(used_out, output.len(), "G{m} consumed full output");
            }
        }
    }

    #[test]
    fn shorthand_simple_balanced_tree() {
        // ((0^3 , 01), (10, (001, 11))) is bin 4. Walk the structure:
        //   root.0 -> (0^3, 01): 0.0 = 0^3 (000), 0.1 = 01
        //   root.1 -> (10, (001,11)): 1.0 = 10, 1.1.0 = 001, 1.1.1 = 11
        let c = ComponentCode::from_shorthand("((0^3 , 01), (10, (001, 11)))");
        let pairs: std::collections::HashMap<Vec<bool>, Vec<bool>> =
            c.pairs().iter().cloned().collect();
        assert_eq!(pairs[&bits("000")], bits("00")); // 03 at path 0,0
        assert_eq!(pairs[&bits("01")], bits("01")); // 01 at path 0,1
        assert_eq!(pairs[&bits("10")], bits("10")); // 10 at path 1,0
        assert_eq!(pairs[&bits("001")], bits("110")); // 001 at path 1,1,0
        assert_eq!(pairs[&bits("11")], bits("111")); // 11 at path 1,1,1
    }

    /// Every shorthand bin (2..=8) parses to a code whose input and
    /// output sets are both prefix-free and the map is a bijection
    /// (encode then decode is the identity on every codeword).
    #[test]
    fn shorthand_bins_are_bijective() {
        let all = bins();
        for bin in &all[1..8] {
            for (input, output) in bin.code.pairs() {
                let (enc_out, used_in) = bin.code.encode_one(input).expect("encode_one");
                assert_eq!(&enc_out, output, "bin {} encode {input:?}", bin.index);
                assert_eq!(used_in, input.len());
                let (dec_in, used_out) = bin.code.decode_one(output).expect("decode_one");
                assert_eq!(&dec_in, input, "bin {} decode {output:?}", bin.index);
                assert_eq!(used_out, output.len());
            }
        }
    }

    #[test]
    fn table10_has_17_bins_with_published_cutoffs() {
        let all = bins();
        assert_eq!(all.len(), 17);
        for (idx, bin) in all.iter().enumerate() {
            assert_eq!(bin.index, (idx + 1) as u8);
            assert_eq!(bin.cutoff_num, CUTOFFS[idx]);
        }
        // Cutoffs are strictly increasing and the last is the full range.
        for w in CUTOFFS.windows(2) {
            assert!(w[0] < w[1], "cutoffs strictly increasing");
        }
        assert_eq!(*CUTOFFS.last().unwrap(), Z_DENOM);
    }

    /// The §IV.D bin-9..17 Golomb assignments G5..G512.
    #[test]
    fn golomb_bins_have_expected_m() {
        let all = bins();
        let expected_m = [5u32, 6, 7, 11, 17, 31, 70, 200, 512];
        for (k, &m) in expected_m.iter().enumerate() {
            let bin = &all[8 + k]; // bins 9..=17 are index 8..=16
            assert_eq!(bin.code.len(), m as usize + 1, "bin {} is G{m}", bin.index);
        }
    }

    /// Bin selection: p = 1/2 lands in bin 1; a p just below the first
    /// cutoff lands in bin 1; p at/above a cutoff steps to the next bin.
    #[test]
    fn bin_selection_intervals() {
        // p = 1/2 = 32768/65536 < 35298 -> bin 1.
        assert_eq!(bin_for_probability(1, 2), 1);
        // p = 35297/65536 < 35298 -> bin 1.
        assert_eq!(bin_for_probability(35297, 65536), 1);
        // p = 35298/65536 = z_1 -> not < z_1, so bin 2.
        assert_eq!(bin_for_probability(35298, 65536), 2);
        // p just below z_2 -> bin 2.
        assert_eq!(bin_for_probability(37344, 65536), 2);
        // p = z_2 -> bin 3.
        assert_eq!(bin_for_probability(37345, 65536), 3);
        // p = 1 -> last bin 17.
        assert_eq!(bin_for_probability(65536, 65536), 17);
        // p just under the final cutoff -> bin 17.
        assert_eq!(bin_for_probability(65392, 65536), 17);
    }

    #[test]
    fn uncoded_bin1_is_identity() {
        let c = ComponentCode::uncoded();
        let (o0, _) = c.encode_one(&bits("0")).unwrap();
        assert_eq!(o0, bits("0"));
        let (o1, _) = c.encode_one(&bits("1")).unwrap();
        assert_eq!(o1, bits("1"));
        let (i0, _) = c.decode_one(&bits("0")).unwrap();
        assert_eq!(i0, bits("0"));
    }

    /// Round-trip the interleaved coder over a fixed (bit, bin) stream:
    /// encode, then decode with the identical per-bit bin assignment, and
    /// recover every source bit exactly (§IV.C end-to-end).
    fn roundtrip(stream: &[(bool, u8)]) {
        roundtrip_counting_flushes(stream);
    }

    /// Round-trip `stream` and return how many mid-stream buffer-full
    /// flushes the encoder performed (so flush-regression tests can
    /// assert they genuinely exercise the path).
    fn roundtrip_counting_flushes(stream: &[(bool, u8)]) -> usize {
        let mut enc = InterleavedEncoder::new();
        for &(b, j) in stream {
            enc.encode_bit(b, j);
        }
        let flushes = enc.mid_stream_flushes;
        let channel = enc.finish();
        let mut dec = InterleavedDecoder::new(&channel);
        for (idx, &(b, j)) in stream.iter().enumerate() {
            let got = dec.decode_bit(j);
            assert_eq!(got, b, "bit {idx} bin {j}: decoded {got} != source {b}");
        }
        flushes
    }

    #[test]
    fn interleave_uncoded_bin_roundtrip() {
        // Bin 1 (uncoded): identity, so the channel mirrors the source.
        let stream: Vec<(bool, u8)> = (0..40).map(|i| ((i * 7 + 3) % 5 < 2, 1u8)).collect();
        roundtrip(&stream);
    }

    #[test]
    fn interleave_single_golomb_bin_roundtrip() {
        // Drive a long run of mostly-zeros through bin 17 (G512), the
        // most aggressive run-length code, then a few ones.
        let mut stream: Vec<(bool, u8)> = Vec::new();
        for _ in 0..1000 {
            stream.push((false, 17));
        }
        for _ in 0..5 {
            stream.push((true, 17));
        }
        for i in 0..50 {
            stream.push((i % 3 == 0, 17));
        }
        roundtrip(&stream);
    }

    #[test]
    fn interleave_shorthand_bin_roundtrip() {
        // Bins 2..=8 each exercise a shorthand-tree code.
        for bin in 2u8..=8 {
            let stream: Vec<(bool, u8)> = (0..200).map(|i| (((i * 13 + 1) % 7) < 5, bin)).collect();
            roundtrip(&stream);
        }
    }

    #[test]
    fn interleave_mixed_bins_roundtrip() {
        // Interleave several bins so the front-of-list ordering and the
        // per-bin partial-word tracking are both exercised.
        let bins_cycle = [1u8, 5, 9, 17, 3, 12, 1, 8];
        let stream: Vec<(bool, u8)> = (0..600)
            .map(|i| {
                let j = bins_cycle[i % bins_cycle.len()];
                let b = ((i * 31 + 7) % 11) < 6;
                (b, j)
            })
            .collect();
        roundtrip(&stream);
    }

    #[test]
    fn interleave_buffer_flush_roundtrip() {
        // Force many open words (alternating bins so each opens a new
        // partial word) to exceed the 2048-word buffer and trigger the
        // §IV.C flush path, then keep decoding correctly.
        let mut stream: Vec<(bool, u8)> = Vec::new();
        for i in 0..(BUFFER_WORDS * 3) {
            // Alternate between two run-length bins; a lone `0` in a
            // Golomb bin starts a partial word that stays open, so the
            // list grows until the flush fires.
            let j = if i % 2 == 0 { 16u8 } else { 17u8 };
            stream.push((false, j));
        }
        roundtrip(&stream);
    }

    /// A deterministic LCG drives many short random (bit, bin) streams
    /// across all 17 bins; every one must round-trip exactly. This
    /// exercises the front-of-list FIFO drain, the per-bin partial-word
    /// tracking, and the end-of-stream flush across a wide variety of
    /// word-completion orderings (the case that surfaced the finish-loop
    /// bug).
    #[test]
    fn interleave_randomized_roundtrip() {
        let mut state = 0x1234_5678u32;
        let mut next = |m: u32| {
            state = state.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            (state >> 8) % m
        };
        for _ in 0..2000 {
            let n = 1 + next(40) as usize;
            let stream: Vec<(bool, u8)> = (0..n)
                .map(|_| {
                    let b = next(2) == 1;
                    let j = (1 + next(17)) as u8;
                    (b, j)
                })
                .collect();
            roundtrip(&stream);
        }
    }

    /// Regression: a **mid-stream** buffer-full flush (§IV.C.1) followed
    /// by more source bits for the flushed bin. The encoder completes
    /// the stuck front word with flush bits and opens a fresh word for
    /// the bin's later bits; the decoder must detect the flush (by
    /// replaying the encoder's word list) and discard the flush bits
    /// instead of serving them as source bits. Before the fix the
    /// decoder desynchronised here — the path was reachable from real
    /// packets once bodies grew past ~2048 words with a stuck
    /// high-Golomb front word.
    #[test]
    fn interleave_mid_stream_flush_resyncs() {
        // A lone zero in bin 17 (G512) opens a word that stays partial
        // for up to 512 bits; single-`1` Golomb words in bin 9 (G5's
        // input `1` is a complete one-bit codeword) pile up complete
        // behind it until the 2048-word buffer forces a front flush.
        let mut stream: Vec<(bool, u8)> = Vec::new();
        stream.push((false, 17));
        for _ in 0..(BUFFER_WORDS + 40) {
            stream.push((true, 9));
        }
        // Post-flush bits for bin 17: the decoder must serve these from
        // the encoder's fresh word, not from the flushed word's flush
        // bits.
        for i in 0..24 {
            stream.push((i % 3 == 0, 17));
            stream.push((i % 5 == 0, 9));
        }
        let flushes = roundtrip_counting_flushes(&stream);
        assert!(flushes > 0, "stream must hit the mid-stream flush path");
    }

    /// Randomized stress over the mid-stream flush path: skewed streams
    /// long enough to hit the 2048-word cap repeatedly, mixing stuck
    /// high-Golomb words with fast-completing tree-code words across
    /// several bins.
    #[test]
    fn interleave_mid_stream_flush_randomized() {
        let mut state = 0xC0FF_EE11u32;
        let mut next = |m: u32| {
            state = state.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            (state >> 8) % m
        };
        let mut total_flushes = 0usize;
        for _ in 0..4 {
            let n = BUFFER_WORDS * 3;
            let stream: Vec<(bool, u8)> = (0..n)
                .map(|_| {
                    // Mostly single-`1` Golomb words (complete in one
                    // bit, so the list genuinely grows) with occasional
                    // zeros into deep Golomb bins so front words get
                    // stuck and flush mid-stream.
                    let j = match next(40) {
                        0 => 17u8,
                        1 => 16,
                        v => 9 + (v % 4) as u8,
                    };
                    // Deep Golomb bins receive zeros (their words stay
                    // partial and block the front); shallow bins mostly
                    // receive ones (complete words pile behind).
                    let b = if j >= 16 { false } else { next(16) != 0 };
                    (b, j)
                })
                .collect();
            total_flushes += roundtrip_counting_flushes(&stream);
        }
        assert!(
            total_flushes > 0,
            "streams must hit the mid-stream flush path"
        );
    }

    #[test]
    fn interleave_empty_stream() {
        let enc = InterleavedEncoder::new();
        let channel = enc.finish();
        // No source bits encoded -> nothing to decode; channel is empty
        // or pure padding.
        assert!(channel.len() <= 1);
        let _dec = InterleavedDecoder::new(&channel);
    }

    #[test]
    fn bin_and_invert_reduction() {
        // p1 = 1/2 -> p0 = 1/2 >= 1/2, no inversion, bin 1.
        assert_eq!(bin_and_invert(1, 2), (1, false));
        // p1 small -> p0 large -> high bin, no inversion.
        let (bin, inv) = bin_and_invert(1, 100);
        assert!(bin > 8 && !inv, "p0 = 99/100 -> high bin no inversion");
        // p1 large -> p0 small (<1/2) -> invert, code with p = p1.
        let (bin2, inv2) = bin_and_invert(99, 100);
        assert!(inv2, "p1 = 99/100 -> invert");
        assert_eq!(bin, bin2, "symmetric: same bin after inversion");
    }

    /// Round-trip the context-driven wrapper with a *fixed* per-bit
    /// probability estimate (encoder + decoder must agree on the estimate
    /// — in the codec both reproduce it from the §III.C model).
    #[test]
    fn ixec_fixed_probability_roundtrip() {
        // A skewed source: mostly zeros, estimate p1 = 1/16.
        let symbols: Vec<u8> = (0..500).map(|i| ((i * 37 + 5) % 16 == 0) as u8).collect();
        let (p1n, p1d) = (1u32, 16u32);
        let mut enc = IxecEncoder::new();
        for &s in &symbols {
            enc.encode_bit(s, p1n, p1d);
        }
        let channel = enc.finish();
        let mut dec = IxecDecoder::new(&channel);
        for (i, &s) in symbols.iter().enumerate() {
            assert_eq!(dec.decode_bit(p1n, p1d), s, "symbol {i}");
        }
    }

    /// Round-trip the wrapper driven by an *adaptive* estimate that both
    /// sides reproduce from the decoded-so-far bits — the way the
    /// bit-plane path uses the §III.C model. The estimate depends only on
    /// previously coded symbols, so encoder and decoder stay in lockstep.
    #[test]
    fn ixec_adaptive_estimate_roundtrip() {
        // A simple per-bit estimator: p1 = (ones_seen + 1) / (total + 2).
        let symbols: Vec<u8> = (0..800)
            .map(|i| (((i * 101 + 13) % 23) < 4) as u8)
            .collect();

        // Encode.
        let mut enc = IxecEncoder::new();
        let (mut ones, mut total) = (0u32, 0u32);
        for &s in &symbols {
            let p1n = ones + 1;
            let p1d = total + 2;
            enc.encode_bit(s, p1n, p1d);
            ones += u32::from(s);
            total += 1;
        }
        let channel = enc.finish();

        // Decode, reproducing the same estimator from decoded bits.
        let mut dec = IxecDecoder::new(&channel);
        let (mut ones_d, mut total_d) = (0u32, 0u32);
        for (i, &s) in symbols.iter().enumerate() {
            let p1n = ones_d + 1;
            let p1d = total_d + 2;
            let got = dec.decode_bit(p1n, p1d);
            assert_eq!(got, s, "symbol {i}");
            ones_d += u32::from(got);
            total_d += 1;
        }
    }

    /// The interleaved coder should compress a skewed source below the
    /// raw bit count (a sanity check that the entropy stage does work,
    /// not just round-trip). A 1000-bit mostly-zeros source must pack to
    /// well under 1000 bits.
    #[test]
    fn ixec_compresses_skewed_source() {
        let symbols: Vec<u8> = (0..1000).map(|i| (i % 50 == 0) as u8).collect();
        let mut enc = IxecEncoder::new();
        for &s in &symbols {
            enc.encode_bit(s, 1, 50); // p1 = 1/50 -> p0 = 49/50, high bin
        }
        let channel = enc.finish();
        assert!(
            channel.len() * 8 < symbols.len(),
            "skewed source compressed: {} bytes for {} bits",
            channel.len(),
            symbols.len()
        );
    }
}
