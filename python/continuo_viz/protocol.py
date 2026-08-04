"""What this viewer and the Rust side have to agree on.

None of it can be derived from anything local: change one side and the other is
silently wrong, because JSON over a network fails by producing nothing rather
than by failing to compile. Keeping it in one module means a change on the Rust
side has a single place to land, instead of being spread through a parser, a
regex, and a subscription string.

Each name below records its Rust counterpart. Those are noted to move between
crates at milestone 7; what matters here is what they say, not which crate they
say it from, so a move is only a correction to these references.
"""

from __future__ import annotations

from enum import Enum

# Root chunk every simulation key sits under: `continuo/{world}/...`.
#
# Rust: `continuo_core::KEY_ROOT`.
KEY_ROOT = "continuo"

# Root the viewer side channel sits under, mirroring `KEY_ROOT` segment for
# segment beneath it: `continuo_viz/{world}/...`.
#
# Rust: `continuo_viz_bridge::VIZ_KEY_ROOT`.
VIZ_KEY_ROOT = "continuo_viz"

# The log header's `version` field. A log written by an older or newer continuo
# is not silently half-understood: parsing refuses it instead.
#
# Rust: written by `continuo_conductor::Recorder`, which has no constant for it.
LOG_VERSION = 1


class MessageType(str, Enum):
    """What kind of thing a payload is, as stated in a sample's metadata.

    Rust: ``continuo_viz_bridge::MessageType``, whose ``snake_case`` serde
    renaming produces the values below.

    Inherits ``str`` so a member compares equal to the raw JSON string and can
    be used directly against a decoded value, without every comparison site
    converting first.

    Milestone 7 adds the tick protocol and the join and leave requests as
    further types. A viewer that predates them should ignore them rather than
    read them as something they are not, which is what :meth:`parse` returning
    ``None`` is for.
    """

    SIM_DATA = "sim_data"
    """A component publishing what it simulated, such as a pose or a command.

    Rust: ``MessageType::SimData``.
    """

    MEMBERSHIP_STATUS = "membership_status"
    """The conductor announcing a membership change it has applied.

    Rust: ``MessageType::MembershipStatus``.
    """

    @classmethod
    def parse(cls, raw: object) -> MessageType | None:
        """The type named by ``raw``, or ``None`` if this viewer has no such type.

        Returning ``None`` rather than raising is deliberate. An unknown type is
        a message from a newer continuo, not corruption, and a live viewer that
        died on the first one would be useless against a world it half
        understands.
        """
        try:
            return cls(raw)
        except ValueError:
            return None
