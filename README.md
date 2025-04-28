# Sonar-3D-15-api-example
API example for reading data from the Water Linked Sonar 3D-15 with Python

# Setup
In your python environment you need to install the requirements.txt 
```consol
pip install -r requirements.txt
```
!!! Note
    It is important to use the 4.24.2 protobuf version in the requirements file. If you already have protbuf installed you need to downgrade or look at alternative 2.

RUN:
```consol
python read_api_sonar_3d_15_data.py
```

Optionally you can add the a `--save` argument to save the data to file:
- RangeImage to 3d voxels in the `.xzy` format (supported by for example MeshLab) in the Sonars frame of reference
- BitmapImage to grayscale image in the `.pgm` format (supported by Photoshop, GIMP etc)

The save function is provided as an example of how to process the data from the Sonar, please modify into your needs.

# Alternative 2

If you are already using another version of protobuf this is no problem, as you can take the 'sonar-3d-15-protocol.proto' file and generate a new 'sonar_3d_15_protocol_pb2.py with the correct version. For this you need to install the protobuf compiler to match your protobuf pip installed version (runner). Please see this doc: https://protobuf.dev/support/cross-version-runtime-guarantee/

## Example of compiling a python file from the proto file
```consol
protoc --python_out=. sonar-3d-15-protocol.proto  
```

