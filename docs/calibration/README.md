# Calibration files

Measured values that describe *this physical rover* and cannot be re-derived
from anything in this repository. They are kept here because losing them
means going back to the hardware with a spanner, not because anything in
`rover/`, `ground_station/` or `sim/` reads them.

## `asteropeEncoderOffsets.json`

The zero positions of Asterope's four steering encoders, in degrees.

**This copy is a backup, not the live file.** The BEMA drive server reads it
from an absolute path on the BEMA Pi (`192.168.178.26`):

```
/home/star/asteropeEncoderOffsets.json
```

opened in `bemacontroller/src/BemaServer.cpp` (the `std::ifstream` near line
310). Editing the copy in this directory changes nothing on the rover; to
change the calibration, edit the file on the Pi and copy the result back
here so the backup stays true.

**When these numbers change:** only when the hardware does — an encoder
remounted, a steering module swapped, a wheel re-zeroed. They belong to the
rover, so they survive re-imaging a ground-station laptop, re-flashing the
Orin, and moving the ground station to another machine entirely. None of
those touch the encoders.

**When to re-measure:** if a wheel's straight-ahead no longer looks straight
ahead, or the wheel view in the ground station disagrees with the chassis in
front of you. Re-zero on the rover, then update both the Pi and this copy.
