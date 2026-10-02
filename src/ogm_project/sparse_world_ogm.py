from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


@dataclass
class SparseWorldOGM:
    chunk_size: int = 256
    default_value: float = 0.0
    chunks: dict[tuple[int, int], np.ndarray] = field(default_factory=dict)

    def _chunk_key(self, ix: int, iy: int) -> tuple[int, int]:
        return ix // self.chunk_size, iy // self.chunk_size

    def _local_index(self, ix: int, iy: int) -> tuple[int, int]:
        return iy % self.chunk_size, ix % self.chunk_size

    def get_chunk(self, ix: int, iy: int) -> np.ndarray:
        key = self._chunk_key(ix, iy)
        if key not in self.chunks:
            self.chunks[key] = np.full(
                (self.chunk_size, self.chunk_size),
                self.default_value,
                dtype=np.float32,
            )
        return self.chunks[key]

    def set_cell(self, ix: int, iy: int, value: float) -> None:
        chunk = self.get_chunk(ix, iy)
        ly, lx = self._local_index(ix, iy)
        chunk[ly, lx] = value

    def get_cell(self, ix: int, iy: int) -> float:
        key = self._chunk_key(ix, iy)
        if key not in self.chunks:
            return self.default_value
        ly, lx = self._local_index(ix, iy)
        return float(self.chunks[key][ly, lx])
