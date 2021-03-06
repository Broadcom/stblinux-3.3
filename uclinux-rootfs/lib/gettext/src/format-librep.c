/* librep format strings.
   Copyright (C) 2001-2002 Free Software Foundation, Inc.
   Written by Bruno Haible <haible@clisp.cons.org>, 2001.

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2, or (at your option)
   any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software Foundation,
   Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.  */

#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "format.h"
#include "xmalloc.h"
#include "error.h"
#include "progname.h"
#include "gettext.h"

#define _(str) gettext (str)

/* librep format strings are implemented in librep-0.14/src/streams.c.
   A directive
   - starts with '%' or '%m$' where m is a positive integer,
   - is optionally followed by any of the characters '-', '^', '0', '+', ' ',
     each of which acts as a flag,
   - is optionally followed by a width specification: a nonempty digit
     sequence,
   - is optionally followed by '.' and a precision specification: a nonempty
     digit sequence,
   - is finished by a specifier
       - '%', that needs no argument,
       - 'c', that need a character argument,
       - 'd', 'x', 'X', 'o', that need an integer argument,
       - 's', that need an argument and prints it using princ,
       - 'S', that need an argument and prints it using prin1.
   Numbered ('%m$') and unnumbered argument specifications can be used in the
   same string. The effect of '%m$' is to set the current argument number to
   m. The current argument number is incremented after processing a directive.
 */

enum format_arg_type
{
  FAT_NONE,
  FAT_CHARACTER,
  FAT_INTEGER,
  FAT_OBJECT_PRETTY,
  FAT_OBJECT
};

struct numbered_arg
{
  unsigned int number;
  enum format_arg_type type;
};

struct spec
{
  unsigned int directives;
  unsigned int numbered_arg_count;
  unsigned int allocated;
  struct numbered_arg *numbered;
};

/* Locale independent test for a decimal digit.
   Argument can be  'char' or 'unsigned char'.  (Whereas the argument of
   <ctype.h> isdigit must be an 'unsigned char'.)  */
#undef isdigit
#define isdigit(c) ((unsigned int) ((c) - '0') < 10)


/* Prototypes for local functions.  Needed to ensure compiler checking of
   function argument counts despite of K&R C function definition syntax.  */
static int numbered_arg_compare PARAMS ((const void *p1, const void *p2));
static void *format_parse PARAMS ((const char *format));
static void format_free PARAMS ((void *descr));
static int format_get_number_of_directives PARAMS ((void *descr));
static bool format_check PARAMS ((const lex_pos_ty *pos,
				  void *msgid_descr, void *msgstr_descr,
				  bool equality,
				  bool noisy, const char *pretty_msgstr));


static int
numbered_arg_compare (p1, p2)
     const void *p1;
     const void *p2;
{
  unsigned int n1 = ((const struct numbered_arg *) p1)->number;
  unsigned int n2 = ((const struct numbered_arg *) p2)->number;

  return (n1 > n2 ? 1 : n1 < n2 ? -1 : 0);
}

static void *
format_parse (format)
     const char *format;
{
  struct spec spec;
  struct spec *result;
  unsigned int number;

  spec.directives = 0;
  spec.numbered_arg_count = 0;
  spec.allocated = 0;
  spec.numbered = NULL;
  number = 1;

  for (; *format != '\0';)
    if (*format++ == '%')
      {
	/* A directive.  */
	enum format_arg_type type;

	spec.directives++;

	if (isdigit (*format))
	  {
	    const char *f = format;
	    unsigned int m = 0;

	    do
	      {
		m = 10 * m + (*f - '0');
		f++;
	      }
	    while (isdigit (*f));

	    if (*f == '$' && m > 0)
	      {
		number = m;
		format = ++f;
	      }
	  }

	/* Parse flags.  */
	while (*format == '-' || *format == '^' || *format == '0'
	       || *format == '+' || *format == ' ')
	  format++;

	/* Parse width.  */
	if (isdigit (*format))
	  {
	    do format++; while (isdigit (*format));
	  }

	/* Parse precision.  */
	if (*format == '.')
	  {
	    format++;

	    if (isdigit (*format))
	      {
		do format++; while (isdigit (*format));
	      }
	  }

	switch (*format)
	  {
	  case '%':
	    type = FAT_NONE;
	    break;
	  case 'c':
	    type = FAT_CHARACTER;
	    break;
	  case 'd': case 'x': case 'X': case 'o':
	    type = FAT_INTEGER;
	    break;
	  case 's':
	    type = FAT_OBJECT_PRETTY;
	    break;
	  case 'S':
	    type = FAT_OBJECT;
	    break;
	  default:
	    goto bad_format;
	  }

	if (type != FAT_NONE)
	  {
	    if (spec.allocated == spec.numbered_arg_count)
	      {
		spec.allocated = 2 * spec.allocated + 1;
		spec.numbered = (struct numbered_arg *) xrealloc (spec.numbered, spec.allocated * sizeof (struct numbered_arg));
	      }
	    spec.numbered[spec.numbered_arg_count].number = number;
	    spec.numbered[spec.numbered_arg_count].type = type;
	    spec.numbered_arg_count++;

	    number++;
	  }

	format++;
      }

  /* Sort the numbered argument array, and eliminate duplicates.  */
  if (spec.numbered_arg_count > 1)
    {
      unsigned int i, j;
      bool err;

      qsort (spec.numbered, spec.numbered_arg_count,
	     sizeof (struct numbered_arg), numbered_arg_compare);

      /* Remove duplicates: Copy from i to j, keeping 0 <= j <= i.  */
      err = false;
      for (i = j = 0; i < spec.numbered_arg_count; i++)
	if (j > 0 && spec.numbered[i].number == spec.numbered[j-1].number)
	  {
	    enum format_arg_type type1 = spec.numbered[i].type;
	    enum format_arg_type type2 = spec.numbered[j-1].type;
	    enum format_arg_type type_both;

	    if (type1 == type2)
	      type_both = type1;
	    else
	      /* Incompatible types.  */
	      type_both = FAT_NONE, err = true;

	    spec.numbered[j-1].type = type_both;
	  }
	else
	  {
	    if (j < i)
	      {
		spec.numbered[j].number = spec.numbered[i].number;
		spec.numbered[j].type = spec.numbered[i].type;
	      }
	    j++;
	  }
      spec.numbered_arg_count = j;
      if (err)
	goto bad_format;
    }

  result = (struct spec *) xmalloc (sizeof (struct spec));
  *result = spec;
  return result;

 bad_format:
  if (spec.numbered != NULL)
    free (spec.numbered);
  return NULL;
}

static void
format_free (descr)
     void *descr;
{
  struct spec *spec = (struct spec *) descr;

  if (spec->numbered != NULL)
    free (spec->numbered);
  free (spec);
}

static int
format_get_number_of_directives (descr)
     void *descr;
{
  struct spec *spec = (struct spec *) descr;

  return spec->directives;
}

static bool
format_check (pos, msgid_descr, msgstr_descr, equality, noisy, pretty_msgstr)
     const lex_pos_ty *pos;
     void *msgid_descr;
     void *msgstr_descr;
     bool equality;
     bool noisy;
     const char *pretty_msgstr;
{
  struct spec *spec1 = (struct spec *) msgid_descr;
  struct spec *spec2 = (struct spec *) msgstr_descr;
  bool err = false;

  if (spec1->numbered_arg_count + spec2->numbered_arg_count > 0)
    {
      unsigned int i, j;
      unsigned int n1 = spec1->numbered_arg_count;
      unsigned int n2 = spec2->numbered_arg_count;

      /* Check the argument names are the same.
	 Both arrays are sorted.  We search for the first difference.  */
      for (i = 0, j = 0; i < n1 || j < n2; )
	{
	  int cmp = (i >= n1 ? 1 :
		     j >= n2 ? -1 :
		     spec1->numbered[i].number > spec2->numbered[j].number ? 1 :
		     spec1->numbered[i].number < spec2->numbered[j].number ? -1 :
		     0);

	  if (cmp > 0)
	    {
	      if (noisy)
		{
		  error_with_progname = false;
		  error_at_line (0, 0, pos->file_name, pos->line_number,
				 _("a format specification for argument %u, as in '%s', doesn't exist in 'msgid'"),
				 spec2->numbered[j].number, pretty_msgstr);
		  error_with_progname = true;
		}
	      err = true;
	      break;
	    }
	  else if (cmp < 0)
	    {
	      if (equality)
		{
		  if (noisy)
		    {
		      error_with_progname = false;
		      error_at_line (0, 0, pos->file_name, pos->line_number,
				     _("a format specification for argument %u doesn't exist in '%s'"),
				     spec1->numbered[i].number, pretty_msgstr);
		      error_with_progname = true;
		    }
		  err = true;
		  break;
		}
	      else
		i++;
	    }
	  else
	    j++, i++;
	}
      /* Check the argument types are the same.  */
      if (!err)
	for (i = 0, j = 0; j < n2; )
	  {
	    if (spec1->numbered[i].number == spec2->numbered[j].number)
	      {
		if (spec1->numbered[i].type != spec2->numbered[j].type)
		  {
		    if (noisy)
		      {
			error_with_progname = false;
			error_at_line (0, 0, pos->file_name, pos->line_number,
				       _("format specifications in 'msgid' and '%s' for argument %u are not the same"),
				       pretty_msgstr,
				       spec2->numbered[j].number);
			error_with_progname = true;
		      }
		    err = true;
		    break;
		  }
		j++, i++;
	      }
	    else
	      i++;
	  }
    }

  return err;
}


struct formatstring_parser formatstring_librep =
{
  format_parse,
  format_free,
  format_get_number_of_directives,
  format_check
};


#ifdef TEST

/* Test program: Print the argument list specification returned by
   format_parse for strings read from standard input.  */

#include <stdio.h>
#include "getline.h"

static void
format_print (descr)
     void *descr;
{
  struct spec *spec = (struct spec *) descr;
  unsigned int last;
  unsigned int i;

  if (spec == NULL)
    {
      printf ("INVALID");
      return;
    }

  printf ("(");
  last = 1;
  for (i = 0; i < spec->numbered_arg_count; i++)
    {
      unsigned int number = spec->numbered[i].number;

      if (i > 0)
	printf (" ");
      if (number < last)
	abort ();
      for (; last < number; last++)
	printf ("_ ");
      switch (spec->numbered[i].type)
	{
	case FAT_CHARACTER:
	  printf ("c");
	  break;
	case FAT_INTEGER:
	  printf ("i");
	  break;
	case FAT_OBJECT_PRETTY:
	  printf ("s");
	  break;
	case FAT_OBJECT:
	  printf ("*");
	  break;
	default:
	  abort ();
	}
      last = number + 1;
    }
  printf (")");
}

int
main ()
{
  for (;;)
    {
      char *line = NULL;
      size_t line_len = 0;
      void *descr;

      if (getline (&line, &line_len, stdin) < 0)
	break;

      descr = format_parse (line);

      format_print (descr);
      printf ("\n");

      free (line);
    }

  return 0;
}

/*
 * For Emacs M-x compile
 * Local Variables:
 * compile-command: "/bin/sh ../libtool --mode=link gcc -o a.out -static -O -g -Wall -I.. -I../lib -I../intl -DHAVE_CONFIG_H -DTEST format-librep.c ../lib/libgettextlib.la"
 * End:
 */

#endif /* TEST */
