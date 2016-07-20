## Kinetoscan UI



### Working with the scan ###

The result of the scanning process is a series of .PLY files. Each file represents a single frame of our capture.

We are now going to work with our scan to produce a mesh, for this we are going to use MeshLab.

1. Open MeshLab
2. Create new project
3. Open the first 20 .PLY files. I suggest you work with no more than 40 frames at a time.

Let's downsample our mesh to be able to work with it better. The default captures contain a lot of data, much of it redundant. Downsampling means that we will end up with fewer points on our mesh so it will make working with it more manageable.

1. Select the first mesh.
2. Go to menu Filters > Sampling > Poisson-disk sampling...
3. Check the box named "Base Mesh Subsampling" (important, otherwise it will complain that you need a mesh)

After doing this with each mesh we will have a vastly reduced number of vertexes. All vertexes will be black after this operation because we have lost the color information in the process.
