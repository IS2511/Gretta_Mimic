# Gretta Mimic

The Mimic program for a robotic interface named Gretta.

This project is designed for Arduino Mega 2560.

I personally use PlatformIO(+Atom) to compile and upload code. A `platformio.ini` file is included.

## The IR remote

Using an IR remote you can record and play move sequences

Assuming the following remote layout:

| | | |
| :---: | :---: | :---: |
|   | ⬆ |   |
| ⬅ | K | ➡ |
|   | ⬇ |   |
| 1 | 2 | 3 |
| 4 | 5 | 6 |
| 7 | 8 | 9 |
| * | 0 | # |

## Controls:

- Press `OK` once at the start to begin

- Hold `#` to change mode (record/play+pause)

  In record mode:
    - Press `OK` to record a frame (autoincrements `frameId`)
    - Press or hold `>` and `<` to change frame number (`frameId`)
    - Press `/\` and `\/` to change frame time

  In play/pause mode:
    - Press `OK` to pause/play
    - Press or hold `>` and `<` to change frame number (`frameId`)

- Hold `*` to check (ping) online motors

- Hold `2` to turn all torque off

- Hold `5` to clear the movie

- Hold `8` to toggle loop playing

- Press `0` to stop any action

- Hold `0` at any time to reset via GPIO `pinReset`+RESET (pull to ground)

 **NOTE**: Disconnect the `pinReset`/RESET pin if uploading code!
