# Garment Measurements

Implementation for the shape sampling and measurement part of the paper Korosteleva, Kesdogan, Kemper, Wenninger, Koller, Zhang, Botsch, Sorkine-Hornung "GarmentCodeData: A Dataset of 3D Made-to-Measure Garments With Sewing Patterns".

## Getting Started

Clone this repository:
```bash
git clone https://github.com/mbotsch/GarmentMeasurements.git
```

Compile this project:
```bash
cd GarmentMeasurements
cmake -S . -B build
cmake --build build
cd build
```

## Dependencies

- cgal
- [FBX SDK 2020.3](https://aps.autodesk.com/developer/overview/fbx-sdk)

### Install Dependencies

#### Linux
Before running CMake,
[download](https://damassets.autodesk.net/content/dam/autodesk/www/files/fbx202037_fbxsdk_gcc_linux.tar.gz)
and extract the FBX SDK into the directory `external/fbxsdk`.

#### MacOS
Install cgal using homebrew.
```bash
brew install cgal
```


## Sample Shapes

```bash
./generate_shapes
```

Change PCA parameters as desired and save the skinned mesh. The mesh is saved in the current work directory as `output.obj`.

## Perform Measurements

```bash
./measurement output.obj measurements.yml
```



