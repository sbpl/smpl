Debug tools
---

Header-only files:
* `marker.h` - Data structures for visualizations.
    * ##### Namespace: `smpl::visual`
    * Shape types markers:
      * Empty
      * Arrow
      * Cube
      * Sphere
      * Ellipse
      * Cylinder
      * LineList
      * LineStrip
      * CubeList
      * SphereList
      * PointList
      * BillboardText
      * MeshResource
      * TriangleList
    * `Color` struct: Contains the following fields:
      * `float r`: Red.
      * `float g`: Green.
      * `float b`: Blue.
      * `float a`: Alpha.

  * `Marker` struct: Contains the following fields:
    * `std::string ns`: Namespace for the marker.
    * `int id`: ID of the marker.
    * `int type`: Type of the marker.
    * `int action`: Action to take for the marker.
    * `geometry_msgs::Pose pose`: Pose of the marker.
    * `geometry_msgs::Vector3 scale`: Scale of the marker.
    * `std_msgs::ColorRGBA color`: Color of the marker.
    * `bool lifetime`: Lifetime of the marker.
    * `bool frame_locked`: Whether the marker should be frame-locked.
    * `std::vector\<geometry_msgs::Point> points`: Points of the marker.
    * `std::vector\<std_msgs::ColorRGBA> colors`: Colors of the marker.
    * `std::string text`: Text of the marker.
    * `std::string mesh_resource`: Mesh resource of the marker.
    * `bool mesh_use_embedded_materials`: Whether to use embedded materials of the marker `Pose` struct and the following 
    attributes:
      * `shape`: Shape of the marker.
      * `color`: Color of the marker.
      * `lifetime`: Lifetime of the marker.
      * `frame_id`: Frame ID of the marker.
      * `ns`: Namespace.
      * `id`: ID.
      * `Action`: Action to take for the marker (Add, Delete, Modify..).
      * `flags`.

---
Files with _source_ scripts:
* `colors` - Functions for dealing with different colors representations
  * ##### Namespace: `smpl::visual`
  * Includes: `marker.h`
    Functions:
  * `Color MakeColorHSV(double h, double s, double v)`: Make a `Color` object from HSV.
  * `void hsv_to_rgb(T* r, T* g, T* b, T h, T s, T v)`: Convert HSV to RGB.
  * `void rgb_to_hsv(T* h, T* s, T* v, T r, T g, T b)`: Convert RGB to HSV.
  **Note:** In both cases one can input `float` or `double` values.
  
* `marker_utils` - Utils for making the markers. 
  * ##### Namespace: `smpl::visual`
  * Includes: `marker.h`, `spatial.h`
    Functions:
  * `Marker MakeEmptyMarker`
  * `Marker MakeCubesMarker`
  * `Marker MakeLineMarker`
  * `std::vector<Marker\> MakeFrameMarkers`
  * `std::vector<Marker\> MakePoseMarkers`

* `visualize` - Visualization structs, functions and aliases definitions.
  * ##### Namespace: `smpl::visual`
  * Includes: `config.h`, `visualization_msgs/MakeArray.h`
  ### TODO: Add more info about the functions and structs.


---

## Usage:
Example of usage of the functionality provided by smpl's debug visualization module and associates 
can be found in the following files:
1. [debug_vis_demo.cpp](../../../smpl_test/src/debug_vis_demo.cpp).
2. [visualizer_test.cpp](../../../smpl_test/src/visualizer_test.cpp).

## TODO: add more info about the usage of the debug tools. Explain specific key points.

---