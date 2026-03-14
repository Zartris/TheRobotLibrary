# Semantic Segmentation Theory

## Task Definition

Semantic segmentation assigns a class label to every pixel in an image:

$$f: \mathbb{R}^{H \times W \times 3} \to \{c_1, \ldots, c_K\}^{H \times W}$$

where $K$ is the number of semantic classes (e.g., ROAD, SIDEWALK, OBSTACLE, PERSON, VEHICLE, BUILDING, SKY, UNKNOWN).

## M23 Scope: Stub-Only

M23 defines the interface (`ISemanticSegmenter`) and ships a stub implementation (`StubSemanticSegmenter`) that returns an all-ROAD label map. No DL inference is performed.

The `PluginSemanticSegmenter` provides a `std::function` injection point for future DL backends.

## DL Backend Integration

To implement a `SegmenterPlugin` wrapping ONNX Runtime:

```cpp
#include <semantic_segmentation/segmenter_plugin.hpp>
// (and your ONNX Runtime headers)

SegmenterPlugin onnxPlugin = [session = std::make_shared<Ort::Session>(...)](const RgbImage& img)
    -> std::expected<SemanticMap, std::string>
{
    // 1. Preprocess: convert RgbImage to float HWC tensor, normalize to [0, 1]
    //    Input format: [H, W, 3] float32, RGB channel order, values in [0, 1]
    // 2. Run ONNX session: session->Run(inputTensor)
    // 3. Postprocess: argmax over class dimension → SemanticMap (H × W grid of SemanticClass)
    //    Output format: class index map [H, W] int32
    return semanticMap;
};

PluginSemanticSegmenter segmenter{onnxPlugin};
```

**Expected model input:** `[1, H, W, 3]` float32 (NHWC), RGB order, normalized to $[0, 1]$.  
**Expected model output:** `[1, H, W]` int32 class index map (one index per pixel matching `SemanticClass` enum).

## References

- Long et al., "Fully Convolutional Networks for Semantic Segmentation," CVPR 2015
- Chen et al., "DeepLab: Semantic Image Segmentation," IEEE TPAMI 2017
