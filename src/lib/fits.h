/*
 * $Source: /nfs/benz/h7/gbc/Picture/Repo/pict/src/fits.h,v $
 *
 * Headers for a trivial FITS library.
 */

#define ROW_PER_BUF 36 /* == 2880/80 */
#define FITS_LINE 80

#ifndef NULL
#define NULL ((void *)0)
#endif /* NULL */

union fits_line {
     char line[FITS_LINE];
      struct {
	    char name[8];
	    char equals;
	    char value[71];
      } field;
};

struct fits_buffer {
      union fits_line fl[36];
};

struct fits_table {
      struct fits_buffer fb;
      int length;
      struct fits_table *next;
};
typedef struct fits_table FHdr;

extern struct fits_table *read_fitsfile(char *file);
extern int write_fitsfile(char *file, struct fits_table *ft);
extern int fits_chg(struct fits_table *ft, char *name, char *val);
extern int fits_add(struct fits_table *ft, char *name, char *val);
extern int fits_coa(struct fits_table *ft, char *name, char *val);

extern int fits_callocs;
extern void output_frame(int c, char *n, FHdr *f, int fn, time_t now);

/*
 * $Log: fits.h,v $
 * Revision 1.7  2006/11/08 18:53:20  gbc
 * added some externs
 *
 * Revision 1.6  2006/10/04 18:27:01  gbc
 * Added fits_coa to change or add in a reasonably efficient way.
 *
 * Revision 1.5  2006/05/11 18:45:59  gbc
 * Canonicalized on Source and Log keywords, removed many LOSERisms.
 *
 * Revision 1.4  2006/05/04 14:55:53  gbc
 * checkpoint--creates fits files
 *
 * Revision 1.3  2006/05/03 21:38:13  gbc
 * ansified, and added check for fits library
 *
 * Revision 1.2  2006/05/03 20:48:04  gbc
 * cleanup for gcc4.0
 *
 * Revision 1.1  2006/04/20 17:20:56  gbc
 * imported originals
 *
 * Revision 1.1  1998/02/26 19:21:20  mlv
 * Initial revision
 */
