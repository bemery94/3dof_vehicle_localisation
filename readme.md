# 3DoF Vehicle Localisation from Multi-View Camera Geometry

## Documentation

* My final thesis can be found [here](documents/brendan_capstone_thesis.pdf).

* A video demonstrating how to create the Blender simulations and run the Matlab script can be found [here](documents/capstone_demo_video.mp4).

* My capstone poster can be found [here](documents/capstone_poster.pptx)

* A video demonstrating the optimisation (Note. that the results shown in this video are old and the current optimisation can handle the "difficult" cases presented in the video. However, the video is added so that the optimisation process can be visualised) can be found [here](documents/capstone_poster_video.mp4)

## Folder Structure

* In order to use the code as it is now, you need to follow the current folder structure for the blender files and rendered images. You will need to provide the paths to these directories in the main matlab script and the original_real.blend and original_sim.blend blender scripts.
* Blender files:

```
​```
blender
│   meters.py    
│
└─── real_and_sim_setups
│   │   original_real.blend
│   │   original_sim.blend
│   
└───van_model/
    │   original_cam_setup.blend
    │   van_model.blend
​```
```

* Rendered images:

```
images    
│
└─── original_real_pics
│    │   cam_<folder_cams>                     # These will be generated automatically
│        │
│        └───x_<x_gt>_y_<y_gt>_r_<theta_gt>    # These will be generated automatically
│            │   
│            └─── image  
│                |   camera_<id>.png
│       
└─── original_sim_pics
│    |   cam_<folder_cams>                     # These will be generated automatically
│        │
│        └───depth                             # These will be generated automatically
│        |   │    depth_<id>.mat
│        |        
│        └───image        
|            |    camera<id>.png
```

## Creating the Simulated Images in Blender

1. Open <path>/blender/van_model/van_model.blend
   1. On the far right of the screen find the Scene tree that contains the branches: RenderLayers, World, Skid, car etc. 
   2. Expand the car branch and you should see Variante01.005 and the material below that. Right click on the material and delete it.
   3. If there are any parts of the vehicle that you want to add or remove, apply the changes to this model.
   4. Save and close the file.
2. Open <path>/blender/van_model/original_cam_setup.blend
   1. On the far right of the screen find the Scene tree that contains the branches: RenderLayers, World, Skid, car etc. 
   2. Click the eye icon next to each camera (1-23) to show/hide the camera. Make sure that only the cameras that you want to use are visible and hide the rest.
   3. Save and close the file.
3. Open <path>/blender/real_and_sim_setups/original_real.blend
   1. In the top menu bar that contains "File", "Render" etc. find the text box to choose the screen layout. It will usually say "Default" in the text box. 
   2. Click on the selection button to the left of the text box and choose "Scripting". The screen layout should change and a scripting window should open to the left.
   3. In the menu panel immediately below the scripting window which contains "View", "Text" etc., find the text box that should say "create_scene" or "render_scene". 
   4. Click on the selection button to the left of the text box and choose "create_scene". Check that the paths in the script are correct.
   5. Click on the "Run Script" button. The vehicle model and the cameras you selected in step 2 should be appear in the main window.
   6. Press the 'n' key once or twice to make sure that the "Transform" tab is open. Under the "Location" and "Rotation" sections, choose the x, y and euler Z values which will be the offset of the simulated "real" car. These are the values that the optimiser will try to estimate.
   7. In the "render_scene" script, you can change "bpy.context.scene.render.resolution_percentage" to scale the resolution while keeping the aspect ratio fixed. E.g. if you set it to 50, the output resolution will be 50% * (2592 x 2048). Remember this value as you will need to set the same for the simulated images.
   8. Click on the selection button again and select "render"_scene". Check that the paths in the script are correct.
   9. Click on the "Run Script" button. The camera images will now render. This may take some time depending on the number of cameras and resolution of the images. If you open the terminal, there should be some information about the rendering that will appear after each image is rendered.
4. Open <path>/blender/real_and_sim_setups/original_sim.blend
   1. Repeat the steps for the original_real.blend model except for step 6. For convenience, leave the simulated at the origin. If you do transform the vehicle, the optimisation will just estimate the relative transform between the real and simulated models.
5. Go to the image directory that was listed in the "render_scene" scripts for both the original_sim.blend and original_real.blend files and check that the images are there.



## Running the Optimsation in Matlab

1. Ensure that Peter Corke's [Robotics Toolbox](http://petercorke.com/wordpress/toolboxes/robotics-toolbox) is installed and is on the Matlab search path. 
2. Run "startup_rvc" to initialise the toolbox.
3. Set "plot_figures" and "show_raw_images" as desired.
4. Set the folder_cams:
   1. This should include all of the cameras you made visible in the Blender model or you can use an existing set that have already been produced. You can find these by checking <path>/images/original_real_pics/cam_<folder_cams> and use any of those by adding all of the cameras listed in the directory name in in the folder_cams array.
5.  Set the cam_id_list:
   1. Select any subset of the cameras specified in folder_cams. This allows you to try using different subsets of cameras during tests without having to re-render every combination in Blender.
6. Set the ground truth offset you used for the model in original_real.blend. This value is used to find the images rendered from original_real.blend.
7. Check that the paths for image_dir_sim and image_dir_real are correct. If the folder structure has been setup correctly, you shouldn't need to make any more changes.
8. In the main while loop, you can set the number of iterations that the optimisation should run for or run until the change in error (delta_error) is smaller than some threshold.
9. In <path>/utilities/calcErrorAndJacobian.m, you can set the number of iterations that the optimisation will run using the huber and bisquare loss functions.
10. Run the script to begin the optimisation. The estimated (yaw, x, y) will be printed at every iteration along with the delta_error.
11. Note. If you have set plot_figures or show_raw_images to true, then I have used the pause commands in between plotting so press any key while each figure is open (while the window is selected) to open the next window.



## Troubleshooting

* If the simulated images look correct (i.e. they are the correct edge images of the vehicle) but there is no associated pointcloud when you run the main script, this indicates that the depth measurements of the model didn't work. This generally occurs due to the material used on the original Blender model. 
  * Check: 
    * 1) In the main Matlab script, set show_raw_images to true and run the main script. You should see 3 windows appear: 2 edge images of the real and simulated images and a third which contains the associated pointcloud of the simulated image. If this is empty, then you have likely encountered this issue. 
    * 2) After running the entire optimisation from the main Matlab script, the optimisation results will only show red points which are the points extracted from the real edge image. There will be no green points which are from the simulation.
  * To fix: 
    * Open van_model.blendand on the far right of the screen find the Scene tree that contains the branches: RenderLayers, World, Skid, car etc. Expand the car branch and you should see Variante01.005 and the material below that. Right click on the material and delete it. You should now re-run the create_scene script in original_sim.blend and check that the vehicle does not have the material applied to it. Then re-render the scene by running render_scene and confirm that the pointcloud
