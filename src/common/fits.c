/*
 * $Source: /nfs/benz/h7/gbc/Picture/Repo/pict/src/fits.c,v $
 *
 * Trivial FITS library
 */

static char __rcsID_fits_C[] = "$Ident:$";

#include "../hed/user_header.h"
#include "../hed/fits.h"

int	fits_callocs = 0;

struct fits_table *
read_fitsfile(char *file)
{
    struct fits_table *ft_first =
	(struct fits_table *)calloc(1, sizeof(struct fits_table));
    struct fits_table *ft_cur = ft_first;
    int fd;

    fits_callocs ++;

    fd = open(file, 0);

    if (fd < 0)
    {
	perror(file);
	return NULL;
    }

    for ( ;; )
    {
	int i;

	if (read(fd, (char *)&ft_cur->fb, sizeof(struct fits_buffer))
	    != sizeof(struct fits_buffer))
	{
	    break;
	}

	for (i=0; i<ROW_PER_BUF; i++)
	    if (strncmp(ft_cur->fb.fl[i].field.name, "END     ", 8) == 0)
		break;
	ft_cur->length = i;

	if (i < ROW_PER_BUF)
	    break;

	ft_cur->next=(struct fits_table *)calloc(1, sizeof(struct fits_table));
	fits_callocs ++;
	if (ft_cur->next == NULL)
	{
	    printf ("Can't allocate space for fits buffer\n");
	    return NULL;
	}

	ft_cur = ft_cur->next;
    }

    close (fd);
    return ft_first;
}

int
write_fitsfile(char *file, struct fits_table *ft)
{
    int fd;

    fd = open(file, O_CREAT|O_WRONLY|O_TRUNC, 0666);

    if (fd < 0)
    {
	perror(file);
	return 1;
    }

    for ( ;; )
    {
	if (write(fd, (char *)&ft->fb, sizeof(struct fits_buffer))
	    != sizeof(struct fits_buffer))
	{
	    perror("write");
	    close(fd);
	    return 1;
	}

	if (ft->length < ROW_PER_BUF)
	    break;

	ft = ft->next;
    }

    close (fd);
    return 0;
}

int
fits_chg(struct fits_table *ft, char *name, char *val)
{
    int i, ln = strlen(name), lv = strlen(val);
    char pd_name[8], pd_val[71];

    memset(pd_name, ' ', 8);
    memcpy(pd_name, name, ln<8?ln:8);

    memset(pd_val, ' ', 71);
    memcpy(pd_val, val, lv<71?lv:71);

    while (ft)
    {
	for (i=0; i<ft->length; i++)
	{
	    if (strncmp(pd_name, ft->fb.fl[i].field.name, 8) == 0)
	    {
		memcpy(ft->fb.fl[i].field.value, pd_val, 71);
		return 0;
	    }
	}

	ft = ft->next;
    }

    return 1;
}

int
fits_add(struct fits_table *ft, char *name, char *val)
{
    int i, ln = strlen(name), lv = strlen(val);
    struct fits_table *ft_first = ft;
    char pd_name[8], pd_val[71];

    memset(pd_name, ' ', 8);
    memcpy(pd_name, name, ln<8?ln:8);

    memset(pd_val, ' ', 71);
    memcpy(pd_val, val, lv<71?lv:71);

    while (ft)
    {
	for (i=0; i<ft->length; i++)
	{
	    if (strncmp(pd_name, ft->fb.fl[i].field.name, 8) == 0)
	    {
		return 1;
	    }
	}

	ft = ft->next;
    }

    /* Now I know it doesn't exist */
    ft = ft_first;
    while (ft->next)
	ft = ft->next;

    /* overwrite END */
    memcpy(ft->fb.fl[ft->length].field.name, pd_name, 8);
    ft->fb.fl[ft->length].field.equals = '=';
    memcpy(ft->fb.fl[ft->length].field.value, pd_val, 71);

    ft->length ++;
    /* If we've run out of space */
    if (ft->length == ROW_PER_BUF)
    {
	ft->next=(struct fits_table *)calloc(1, sizeof(struct fits_table));
	fits_callocs ++;
	if (ft->next == NULL)
	{
	    printf ("Can't allocate space for fits buffer\n");
	    return 1;
	}
	ft = ft->next;
	memset((char *)&ft->fb, ' ', sizeof(ft->fb));
    }

    memcpy(ft->fb.fl[ft->length].line, "END", 3);
    /* ft->length++; */
    ft->next = 0;

    return 0;
}

int
fits_coa(struct fits_table *ft, char *name, char *val)
{
    int i, ln = strlen(name), lv = strlen(val);
    struct fits_table *ft_first = ft;
    char pd_name[8], pd_val[71];

    memset(pd_name, ' ', 8);
    memcpy(pd_name, name, ln<8?ln:8);

    memset(pd_val, ' ', 71);
    memcpy(pd_val, val, lv<71?lv:71);

    while (ft)
    {
	for (i=0; i<ft->length; i++)
	{
	    if (strncmp(pd_name, ft->fb.fl[i].field.name, 8) == 0)
	    {
		memcpy(ft->fb.fl[i].field.value, pd_val, 71);
		return 0;
	    }
	}

	ft = ft->next;
    }

    /* Now I know it doesn't exist */
    ft = ft_first;
    while (ft->next)
	ft = ft->next;

    /* overwrite END */
    memcpy(ft->fb.fl[ft->length].field.name, pd_name, 8);
    ft->fb.fl[ft->length].field.equals = '=';
    memcpy(ft->fb.fl[ft->length].field.value, pd_val, 71);

    ft->length ++;
    /* If we've run out of space */
    if (ft->length == ROW_PER_BUF)
    {
	ft->next=(struct fits_table *)calloc(1, sizeof(struct fits_table));
	fits_callocs ++;
	if (ft->next == NULL)
	{
	    printf ("Can't allocate space for fits buffer\n");
	    return 1;
	}
	ft = ft->next;
	memset((char *)&ft->fb, ' ', sizeof(ft->fb));
    }

    memcpy(ft->fb.fl[ft->length].line, "END", 3);
    /* ft->length++; */
    ft->next = 0;

    return 0;
}

#if FITS_TEST_CODE
#include <strings.h>

static char	x_from_file[1024] =   __FILE__ , *from_file = x_from_file;
static char	x_into_file[1024] = "test.fits", *into_file = x_into_file;
static char	identical[4096];

int
main(int ac, char **av)
{
    struct fits_table *ft;
    char *s = rindex(from_file, '/');
    /* from: ../../pict/src/fits.c
       into: ../../pict/data/fits
     */
    if (s) strcpy(s - 3, "data/fits");
    
    if (ac>1) from_file = av[1];
    if (ac>2) into_file = av[2];

    ft = read_fitsfile(from_file);
    if (!ft) {
	printf ("read failed\n");
	return(2);
    }
    if (write_fitsfile(into_file, ft)) {
	printf ("didn't make it\n");
	return(1);
    }
    sprintf(identical, "cmp %s %s", from_file, into_file);
    return(system(identical));
}
#endif /* FITS_TEST_CODE */

/*
 * $Log: fits.c,v $
 * Revision 1.12  2006/11/09 01:47:18  gbc
 * fixed typo
 *
 * Revision 1.11  2006/11/09 01:43:09  gbc
 * more string carefulness
 *
 * Revision 1.10  2006/11/07 19:51:45  gbc
 * check strlen before blasting in
 *
 * Revision 1.9  2006/11/03 02:00:27  gbc
 * Added tracking of allocs on fitsfile.
 *
 * Revision 1.8  2006/11/02 02:13:05  gbc
 * fixed the END treatment and linked list
 *
 * Revision 1.7  2006/10/04 18:50:44  gbc
 * make sure there actually *is* an END
 *
 * Revision 1.6  2006/10/04 18:27:00  gbc
 * Added fits_coa to change or add in a reasonably efficient way.
 *
 * Revision 1.5  2006/09/06 15:58:29  gbc
 * minor nits for APPLE compilation
 *
 * Revision 1.4  2006/05/11 18:45:58  gbc
 * Canonicalized on Source and Log keywords, removed many LOSERisms.
 *
 * Revision 1.3  2006/05/03 21:38:13  gbc
 * ansified, and added check for fits library
 *
 * Revision 1.2  2006/05/03 20:48:04  gbc
 * cleanup for gcc4.0
 *
 * Revision 1.1  2006/04/20 17:20:55  gbc
 * imported originals
 *
 * Revision 1.1  1998/02/26 19:21:20  mlv
 * Initial revision
 *
 */
