These (Seville) world-model centered paths were generated from the
data in the folder 'Ant Data/mosaic-warped'. The files in that folder
were combined with the *labels.csv files from the folder 'Ant
Data/mosaic' to generate this files with a script along the lines of:

```bash
#!/bin/bash

for i in *-warped.csv; do
    ./to-world-coords.py $i "-14,-14" "14,14" > $i.world.csv
done
```

See also the folder 'Ant Data/paths in model space' which contains
similar files for Ants 3, 6, 11 and 12 only (those that were included
in the Sept. 2026 paper).