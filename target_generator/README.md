# Industrial Calibration Target Generator
This target generator is a modified version of [OpenCV's pattern generator](https://github.com/opencv/opencv/tree/master/doc/pattern_tools).

## Commands and Usage
```bash
$ python gen_pattern.py -o mcircles_7x5 -r 7 -c 5 -T mcircles -s 30 -D 15 -a a4
$ inkscape mcircles_7x5.svg --export-area-page --export-area-snap --export-png=mcircles_7x5.png --export-background="rgb(255,255,255)"
```