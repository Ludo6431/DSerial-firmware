/*
 * ihex2bin - Intel-hex format to binary and vice versa converter
 *
 * Copyright (C) 2003  B. Stultiens
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <string.h>
#include <ctype.h>

static int line_number = 0;
static int filler = 0xff;
static int verbose = 0;

static char *read_line(FILE *fp)
{
	static char *line = NULL;
	static int naline = 0;
	int nline;
	char *cptr;

	if(!line)
	{
		naline = 256;
		nline = 0;
		line = malloc(256);
		if(!line)
		{
			fprintf(stderr, "Out of memory\n");
			exit(1);
		}
	}
	cptr = line;
	nline = 0;
	while(1)
	{
		if(!fgets(cptr, naline-nline, fp))
			return NULL;
		nline = strlen(line);
		if(!nline || line[nline-1] != '\n')
		{
			if(feof(fp))
			{
				fprintf(stderr, "Missing newline in line %d\n", line_number);
				exit(1);
			}
			if(nline >= naline-1)
			{
				naline += 256;
				line = realloc(line, naline);
				if(!line)
				{
					fprintf(stderr, "Out of memory\n");
					exit(1);
				}
			}
			cptr = line + nline;
			continue;
		}
		line[nline-1] = '\0';
		return line;
	}
}

#define CHARTOHEX(c)	(((c) >= '0' && (c) <= '9') ? ((c) - '0') : (((c) > 'F') ? ((c) - 'a' + 0x0a) : ((c) - 'A' + 0x0a)))

static int get_byte(const char *s)
{
	if(isxdigit(s[0]) && isxdigit(s[1]))
		return (CHARTOHEX(s[0]) << 4) + CHARTOHEX(s[1]);
	fprintf(stderr, "Non hex digit in line %d\n", line_number);
	exit(1);
}

static int get_word(const char *s)
{
	if(isxdigit(s[0]) && isxdigit(s[1]) && isxdigit(s[2]) && isxdigit(s[3]))
		return (CHARTOHEX(s[0]) << 12) + (CHARTOHEX(s[1]) << 8) + (CHARTOHEX(s[2]) << 4) + CHARTOHEX(s[3]);
	fprintf(stderr, "Non hex digit in line %d\n", line_number);
	exit(1);
}

static void do_hex2bin(FILE *infp, FILE *outfp)
{
	char *line;
	unsigned char *buffer;
	int bytecount = 0;
	int addr;
	int loaddr = 65536;
	int hiaddr = 0;
	int size;
	int mark;
	int i;
	int sum;
	int chk;
	int len;

	buffer = malloc(65536);
	if(!buffer)
	{
		fprintf(stderr, "Out of memory\n");
		exit(1);
	}
	memset(buffer, filler, 65536);

	while(1)
	{
		line_number++;
		line = read_line(infp);
		if(!line)
			break;
		if(line[0] != ':')
		{
			fprintf(stderr, "Missing ':' in line %d\n", line_number);
			exit(1);
		}
		len = strlen(line);
		if(len < 9)
		{
			fprintf(stderr, "Malformed line at %d\n", line_number);
			exit(1);
		}
		size = get_byte(line+1);
		addr = get_word(line+3);
		mark = get_byte(line+7);
		if(len < (size+1) * 2 + 9)
		{
			fprintf(stderr, "Line size is %d, expected %d in line %d\n", len, (size+1)*2+9, line_number);
			exit(1);
		}
		sum = size + mark + (addr >> 8) + (addr & 0xff);
		for(i = 0; i < size; i++)
		{
			int c = get_byte(line+9 + 2*i);
			sum += c;
			if(addr >= 65536)
			{
				fprintf(stderr, "Address space overflow (> 65536) in line %d\n", line_number);
				exit(1);
			}
			if(addr < loaddr)
				loaddr = addr;
			if(addr > hiaddr)
				hiaddr = addr;
			buffer[addr] = (unsigned char)c;
			addr++;
			bytecount++;
		}
		chk = get_byte(line+9 + size*2);
		if(chk != ((-sum) & 0xff))
		{
			fprintf(stderr, "Checksum error 0x%02x, expected 0x%02x in line %d\n", chk, (-sum)&0xff, line_number);
			exit(1);
		}
		if(mark)
			break;
	}
	if(verbose)
	{
		fprintf(stderr, "Total bytes %d (0x%04x)\n", bytecount, bytecount);
		fprintf(stderr, "Low address : 0x%04x (%d)\n", loaddr, loaddr);
		fprintf(stderr, "High address: 0x%04x (%d)\n", hiaddr, hiaddr);
	}
	fwrite(buffer+loaddr, 1, hiaddr - loaddr + 1, outfp);
	free(buffer);
}

static void do_bin2hex(FILE *infp, FILE *outfp)
{
	char buf[16];
	unsigned addr = 0;

	while(!feof(infp))
	{
		int len = fread(buf, 1, sizeof(buf), infp);
		int i;
		int sum = (len & 0xff) + (addr & 0xff) + ((addr >> 8) & 0xff);
		if(ferror(infp))
		{
			perror("read");
			exit(1);
		}
		if(!len)
			continue;
		fprintf(outfp, ":%02X%04X00", len & 0xff, addr & 0xffff);
		for(i = 0; i < len; i++)
		{
			fprintf(outfp, "%02X", buf[i] & 0xff);
			sum += buf[i];
			addr++;
		}
		fprintf(outfp, "%02X\n", (-sum) & 0xff);
	}
	fprintf(outfp, ":00000001FF\n");
}

static const char version_str[] =
	"Intel-hex to bin and vice versa converter. Version 1.0.0 (22-Jun-2003)\n"
	"Copyright (c) 2003 B. Stultiens, published under GNU GPL v2\n"
	;

static const char usage_str[] =
	"Usage: ihex2bin [options] [file]\n"
	"  -f b       Use b as filler byte (default 0xff)\n"
	"  -h         This message\n"
	"  -H         Intel-hex output (bin input)\n"
	"  -o file    Write output to file\n"
	"  -v         Verbose output\n"
	"  -V         Output version and quit\n"
	;

int main(int argc, char *argv[])
{
	int optc;
	int lose = 0;
	char *outname = NULL;
	int bin2hex = 0;
	FILE *infp;
	FILE *outfp;

	while((optc = getopt(argc, argv, "f:hHo:vV")) != EOF)
	{
		switch(optc)
		{
		case 'f':
			filler = strtol(optarg, NULL, 0);
			break;
		case 'h':
			fprintf(stdout, "%s", usage_str);
			return 1;
		case 'H':
			bin2hex = 1;
			break;
		case 'o':
			outname = strdup(optarg);
			if(!outname)
			{
				fprintf(stderr, "Out of memory\n");
				return 1;
			}
			break;
		case 'v':
			verbose = 1;
			break;
		case 'V':
			fprintf(stdout, "%s", version_str);
			return 1;
		default:
			lose++;
			break;
		}
	}
	if(lose)
	{
		fprintf(stderr, "%s", usage_str);
		return 1;
	}

	if(optind < argc)
	{
		infp = fopen(argv[optind], "rb");
		if(!infp)
		{
			perror(argv[optind]);
			return 1;
		}
	}
	else
		infp = stdin;

	if(outname)
	{
		outfp = fopen(outname, "wb");
		if(!outfp)
		{
			perror(outname);
			return 1;
		}
	}
	else
		outfp = stdout;

	if(bin2hex)
		do_bin2hex(infp, outfp);
	else
		do_hex2bin(infp, outfp);

	fclose(infp);
	if(outname)
		fclose(outfp);
	return 0;
}

