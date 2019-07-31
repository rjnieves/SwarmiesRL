"""Amalgamation for the events package.
"""

from .emitter import EventEmitter
from .base import BaseEvent
from .swarmieloc import SwarmieLocEvent
from .cubespotted import CubeSpottedEvent
from .newcube import NewCubeEvent
from .pickup import CubePickedUpEvent
from .dropped import CubeDroppedEvent
from .collected import CubeCollectedEvent
from .futilecollect import FutileCollectAttemptEvent

# vim: set ts=2 sw=2 expandtab:
