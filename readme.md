# UltimateALPR Project

This project is an Automatic License Plate Recognition (ALPR) system that processes images from RTSP cameras to recognize license plates using OpenCV and other image processing techniques.

## Features

- Real-time processing of images from RTSP cameras
- Detection and recognition of license plates
- Configurable search areas and settings for multiple cameras
- Modular and clean code structure for easy maintenance and development

## Project Structure

```alpr_project/
├── CMakeLists.txt
├── src/
│ ├── main.cpp
│ ├── helper/
│ │ ├── definitions.h
│ │ ├── Helper.cpp
│ │ └── Helper.h
│ ├── platerecognizer/
│ │ ├── recognizertool.cpp
│ │ ├── recognizertool.h
│ │ ├── DetectPlates.cpp
│ │ ├── DetectPlates.h
│ │ ├── PossiblePlate.cpp
│ │ ├── PossiblePlate.h
│ │ ├── Preprocess.cpp
│ │ ├── Preprocess.h
│ │ └── DetectChars.cpp
│ │ └── DetectChars.h
│ ├── core/
│ │ ├── Worker.cpp
│ │ ├── Worker.h
│ │ ├── PlateWorker.cpp
│ │ └── PlateWorker.h
│ ├── cardetection/
│ │ ├── cardetection.cpp
│ │ ├── cardetection.h
│ │ └── Blob.cpp
│ │ └── Blob.h
└── alpr_config/
├── plate_list.txt
├── camera_1_search_area.txt
├── camera_2_search_area.txt
├── camera_1_settings.txt
└── camera_2_settings.txt```


## Installation

1. **Clone the repository:**
    ```sh
    git clone https://github.com/username/UltimateALPR.git
    cd UltimateALPR
    ```

2. **Create a build directory and compile the project:**
    ```sh
    mkdir build
    cd build
    cmake ..
    make
    ```

3. **Run the application:**
    ```sh
    ./alpr_project
    ```

## Configuration

Configuration files for the cameras and plate lists are located in the `alpr_config/` directory. Update these files to match your camera settings and plate recognition requirements.

## Contributing

Contributions are welcome! Please open an issue or submit a pull request.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
