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

import json
import logging
import re
from enum import Enum

logger = logging.getLogger(__name__)

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


class UnsupportedLogVersion(Exception):
    """A file is not a log this viewer can read."""


def check_log_header(line: str) -> None:
    """Raises unless ``line`` is a header declaring a version this viewer reads.

    Both halves of the rule are here, since they are one rule: a file with no
    header is refused exactly like a file whose header names a version we do
    not know. Splitting them let a headerless file past a gate that a
    misversioned one could not pass, which is the same as having no gate.
    """
    try:
        header = json.loads(line)
    except json.JSONDecodeError:
        header = None
    if not isinstance(header, dict) or "version" not in header:
        raise UnsupportedLogVersion(
            'not a recorded log: the first line is not a {"version": ...} header'
        )
    version = header["version"]
    if version != LOG_VERSION:
        raise UnsupportedLogVersion(
            f"log declares version {version!r}, this viewer reads {LOG_VERSION}"
        )


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
    """The conductor announcing a membership change it has processed.

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
            logger.debug("no message type named %r; sample ignored", raw)
            return None


# `continuo/{world}/actor/{name}/{signal}`. Anchored at both ends so a longer
# key cannot match by accident.
_ACTOR_KEY = re.compile(
    rf"^{re.escape(KEY_ROOT)}/[^/]+/actor/(?P<actor>[^/]+)/(?P<signal>[^/]+)$"
)


def parse_actor_key(key: str) -> tuple[str, str] | None:
    """Splits an actor key into its actor name and signal name.

    ``continuo/demo/actor/ego/pose`` gives ``("ego", "pose")``.

    Returns ``None`` for anything that is not an actor key, which is how
    world-level traffic and conductor notifications are ignored without
    listing them.
    """
    match = _ACTOR_KEY.match(key)
    if match is None:
        return None
    return match["actor"], match["signal"]
