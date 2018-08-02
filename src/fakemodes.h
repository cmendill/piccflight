/*************************************************
 * Fake Modes
 *************************************************/
enum fakemodes{ FAKEMODE_NONE,
		FAKEMODE_GEN_IMAGE_TIMER_SYNC,   //Generate fake images, synced to a timer
		FAKEMODE_READ_IMAGE_TIMER_SYNC,  //Read fake images from disk, synced to a timer
		FAKEMODE_GEN_IMAGE_CAMERA_SYNC,  //Generate fake images, synced to a timer
		FAKEMODE_READ_IMAGE_CAMERA_SYNC, //Read fake images from disk, synced to a timer
		FAKEMODE_TM_TEST_PATTERN,        //Generate TM test pattern
                NFAKEMODES};
		
