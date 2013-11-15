Image Capture component. 

Processing pipeline:

	Set up
		* Clean the field from all objects
		* Press 'm' on the image display window to start specifying the corners
                  of the field
		* Corners are specified clockwise from top-left:  top-left, top-right, bottom-right, bottom-left
		* Use the keys w,a,s,d to move the cursor by 1 pixel up, left, down, or right respectively
		* Use the keys W,A,S,D to move by 5 pixels in the corresponding direction
		* To record a corner, press space
		* Once the 4 corners are recorded, the image rectification matrix will be estimated, and a
                  background image will be recorded.
		* Once this is done, the image should go black.

	Main image processing loop for robotSoccer

		* grabs a frame from the camera
		* rectifies the image to obtain a rectangular field (size 1024x768 pixels, stored in fieldIm[] which
                  is an array of 1024*768*3 unsigned chars).
		* fieldIm is then processed to remove the background, leaving (ideally) only objects moving on the field.
		  After background subtraction, any pixels in fieldIm with non-zero values should correspond to objects.
		* To access a given pixel's (i,j) RGB within fieldIm, use 
			fieldIm[(i+(j*1024))*3)+0]	-> R component
			fieldIm[(i+(j*1024))*3)+1]	-> G component
			fieldIm[(i+(j*1024))*3)+2]	-> B component
		* The background-subtracted image is then passed on to a blob detector and a blob tracker.
		* The blob detection process creates a labeled image 'labIm' in which pixels for each blob are
		  labeled with some integer value. This may be useful for you later if you need to find which
                  pixels correspond to which blob in the blob list.
		* The final result usable by your AI is a linked list of blobs. The pointer 'tracked_blobs'
		  points to the head of this list.
			* Blob data structure is described in imageCapture.h, but in brief it contains:
				- A blob label so you can find the pixels for this blob in blobIm (you can safely
				  ignore this unless you want to look at the pixels for some reason)
				- A unique blobID. This is assigned the first time a blob is detected, and is the blob
				  is tracked over several frames, the blobID remains the same. 
				  * Note : Blobs are never actually removed from the list, deleted blobs are given a
				           negative blobID. This saves time managing the tracked blob list.
				  Conversely, note that blob labels change on each frame since they depend on how
				  many other blobs are found per frame.
				  * Use the blobID * to identify blobs. Do not use label!
				- Blob position at the current cx[0],cy[0] as well as the past 4 frames if the blob
				  is being tracked (cx[1 to 4] and cy[1 to 4] are only meaningful is the blob is being
                                  tracked!)
				- Blob motion vector [mx,my] which is a smoothed estimate of the current blob's velocity vector
				- Blob's instantaneous velocity vectors for the current and past 4 frames vx[] and vy[]
				  * Note mx,my,vx[] and vy[] are only valid if the blob is being tracked
				- Blob bounding box as top-left, bottom-right
				- Average blob colour as an RGB value. Use this to distinguish bots from balls!
				- Tracked status : 1 if the blob is being tracked (has been identified over at least 2 frames)
						   0 if the blob is new, or if a matching blob has not been found within the
						     search window
				  * Note: tracked status = 0 can easily mean that the tracker lost the blob because it moved
							   too quickly between frames. Your AI should be able to figure this
							   out. Also, tracked status = 0 may result when blobs merge from one
							   frame to the next. Your AI will also need to handle this
				- Blob age : The number of frames the blob has been tracked, or the number of frames that the
					     blob has remained un-tracked. Un-tracked blobs are removed from the list after	
					     30 frames.
				- You should ignore 'updated' - this is a flag used by the blob tracking code.

		* Your code should NOT change the tracked_blobs list. However, I expect your AI will need to keep its own record
			of which blobs correspond to what stuff in the world. Use the blob list wisely!

	Remember:

		* There is a manual override for the NXT bot. If something goes wrong with your AI, press 'o' for all-stop before
		  damaging your NXT.
		* Pressing 'q' to quit will also send an all-stop to the NXT.
		* Keyboard override:
			'i' - Move forward (toggle forward on/off)
			'j' - Spin left (toggle spin on/off)
			'l' - Spin right (toggle spin on/off)
			'k' - Reverse (toggle reverse on/off)
			'o' - All stop!
