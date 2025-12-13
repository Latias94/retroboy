/// Kind of instruction currently being executed via the micro-step API.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MicroInstrKind {
    None,
    CallA16,
    Ret,
    RetCond,
    JpA16,
    JpCondA16,
    LdSpA16,
    Rst,
    Jr,
    JrCond,
    CallCond,
    /// LDH A,(a8)
    LdhAFromA8,
    /// LDH (a8),A
    LdhAToA8,
}
