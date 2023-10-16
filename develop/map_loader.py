import glob
import os
import re

class Map():
    def __init__(self, folder_path: str, file_extension: str):

        self._folder_path = folder_path
        self._file_extension = file_extension
        self.map3d = []
        self.map_heigh = []

        self._parse_to_object()

    def _parse_to_object(self):
        files = glob.glob(os.path.join(self._folder_path, '*.' + self._file_extension))
        for file_path in files:
            print(file_path)
            # Parse height from file name
            match = re.search(r'\d+', file_path)
            if match:
                self.map_heigh.append(float(match.group()))
            else:
                raise ValueError('Unable to parse map height from file: ' + str(file_path))
            
            # Read file and parse to 3D array
            with open(file_path, 'r') as file:
                data = file.read()

                lines = data.split("\n")
                metadata = {}
                current_pixel_row = []
                map2d = []

                # Loop through lines and parse data
                for line in lines:
                    # Skip comments
                    if line.startswith("#"):
                        continue
                    # Check for magic number P2
                    elif line == "P2":
                        metadata["type"] = "P2"
                    # Check for width and height
                    elif "width" not in metadata:
                        metadata["width"], metadata["height"] = map(int, line.split())
                    # Check for max gray value
                    elif "max_gray" not in metadata:
                        metadata["max_gray"] = int(line)
                    # Parse pixel data
                    else:
                        current_pixel_row = list(map(int, line.split()))
                        map2d.append(current_pixel_row)
            self.map3d.append(map2d)


def load_maps(folder_path: str, file_extension: str):
    map_obj = Map(folder_path, file_extension)
    print('3D lenght: ' + str(len(map_obj.map3d)))
    print('2D lenght: ' + str(len(map_obj.map3d[0])))
    print('1D lenght: ' + str(len(map_obj.map3d[0][0])))
    print('Map heights: ' + str(map_obj.map_heigh))


if __name__ == "__main__":
    load_maps('/home/lrs-ubuntu/Documents/lrs-git/LRS/src/map_loader/src/Maps_2D', 'pgm')