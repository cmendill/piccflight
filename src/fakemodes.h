/*************************************************
 * Fake Modes
 *************************************************/
enum fakemodes{ FAKEMODE_NONE,
		FAKEMODE_GEN_IMAGE_TIMER_SYNC,   //Generate fake images, synced to a timer
		FAKEMODE_READ_IMAGE_TIMER_SYNC,  //Read fake images from disk, synced to a timer
		FAKEMODE_GEN_IMAGE_CAMERA_SYNC,  //Generate fake images, synced to a timer
		FAKEMODE_READ_IMAGE_CAMERA_SYNC, //Read fake images from disk, synced to a timer
		FAKEMODE_TM_TEST_PATTERN,        //Generate TM test pattern
		FAKEMODE_LYTPIX2ALPZER_REFIMG,   //Send LYTPIX2ALPZER reference image
		FAKEMODE_LYTPIX2ALPACT_REFIMG,   //Send LYTPIX2ALPACT reference image
                NFAKEMODES};
		
