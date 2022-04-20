/*************************************************
 * Fake Modes
 *************************************************/
enum fakemodes{ FAKEMODE_NONE,
		FAKEMODE_TEST_PATTERN,  //Generate fake data using a test pattern
		FAKEMODE_TEST_PATTERN2, //Generate fake data using a 2nd test pattern
		FAKEMODE_TEST_PATTERN3, //Generate fake data using a 3rd test pattern
		FAKEMODE_LYT_REFIMG,    //Send current LYT reference image
		FAKEMODE_SCI_MASK,      //Mask SCI image with pixel selection mask
		FAKEMODE_SCI_PROBE,     //Read fake HOWFS probe images
		FAKEMODE_IMREG,         //Image registration pattern
		FAKEMODE_PHASE,         //SCI phase flattening images
                NFAKEMODES};
		
