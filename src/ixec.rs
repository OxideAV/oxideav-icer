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
//! The interleaving machinery (the 2048-word circular buffer, flush
//! bits, decode bookkeeping) lives in a later milestone; this module
//! provides the verified component-code layer it builds on.

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
}
