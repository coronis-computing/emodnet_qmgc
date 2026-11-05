# API reference (Doxygen)

The C++ API is documented in the source code with Doxygen comments. The generated reference is published together with
this site:

**[Open the API reference :material-open-in-new:](https://coronis-computing.github.io/emodnet_qmgc/api/)**

It contains the documentation of every class (e.g., `QuantizedMeshTiler`, `TinCreationStrategy`,
`qmgc::RasterTiler`), together with the file and namespace listings.

## Building it locally

Install `doxygen`, then, from the `build` folder:

```bash
make doc
```

The HTML is written to `build/docs/html/index.html`.

To build it as the documentation workflow does, without configuring the project with CMake (useful when the C++
dependencies are not installed):

```bash
sed "s|@CMAKE_CURRENT_SOURCE_DIR@|$PWD|g" doxyfile.cfg > /tmp/doxyfile
( cat /tmp/doxyfile; echo "OUTPUT_DIRECTORY=site"; echo "HTML_OUTPUT=api"; echo "GENERATE_LATEX=NO" ) | doxygen -
```

## Writing documentation comments

Document new classes and functions in the headers, following the existing style:

```cpp
/**
 * @class TinCreationGreedyInsertionStrategy
 * @brief Creates a TIN using the greedy insertion algorithm of Garland and Heckbert
 */
class TinCreationGreedyInsertionStrategy : public TinCreationStrategy
{
public:
    /**
     * @brief Constructor
     * @param approxTolPerZoom Maximum height error allowed, per zoom (see the per-zoom rules)
     */
    TinCreationGreedyInsertionStrategy(const std::vector<FT>& approxTolPerZoom, ...);
};
```

The API reference is aimed at developers. Document user-facing behavior (options, workflows) in the user guide pages of
this site instead.
