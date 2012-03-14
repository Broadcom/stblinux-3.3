#!/usr/bin/perl -w

use Getopt::Std;
use POSIX;
use Fcntl;

$infile = "";
$outfile = "";
$verbose = 0;

sub run($)
{
	my($cmdline) = @_;

	if($verbose) {
		print " + $cmdline\n";
	}

	my @args = split(/\s+/, $cmdline);
	my $pid = fork();

	if($pid == 0) {
		exec(@args);
	} elsif($pid < 0) {
		die "can't fork: $!";
	}

	wait();
	my $ret = WEXITSTATUS($?);
	if($ret != 0)
	{
		die "command '$cmdline' exited with status $ret";
	}
	return(0);
}

sub dbg($)
{
	my($msg) = @_;
	if($verbose) {
		print $msg;
	}
}

#
# MAIN
#

my %opt = ( );
getopts("vs", \%opt);

$infile = $ARGV[0];
$outfile = $ARGV[1];

if (!defined($infile)) {
	print "usage: elf2bin.pl [ -v ] [ -s ] infile [ outfile ]\n";
	print "\n";
	print "Options:\n";
	print "  -v             Verbose output\n";
	print "  -s             Use 'srec' format instead of raw binary\n";
	exit 1;
}

my $outfmt = "binary";
my $def_outfile = "vmlinux.bin";

if (defined($opt{'s'})) {
	$outfmt = "srec";
	$def_outfile = "vmlinux.srec";
}

if (defined($opt{'v'})) {
	$verbose = 1;
}

if (!defined($outfile)) {
	$outfile = $def_outfile;
}

my $hdr;

open(F, "<$infile") or die;
sysread(F, $hdr, 0x40) or die;
close(F);

my @elf = unpack("NC12", $hdr);
if (($elf[0] & 0xffffff00) == 0x1f8b0800) {
	dbg("Unpacking gzipped image\n");

	unlink("elf2bin.tmp");
	unlink("elf2bin.tmp.gz");
	symlink($infile, "elf2bin.tmp.gz");

	run("gunzip -f elf2bin.tmp.gz");

	$infile = "elf2bin.tmp";
	open(F, "<$infile") or die;
	sysread(F, $hdr, 0x40) or die;
	close(F);
	@elf = unpack("NC12", $hdr);
}

if ($elf[0] != 0x7f454c46) {
	die "Invalid ELF header: ELFMAG != 0x7f454c46";
}

if ($elf[1] != 1) {
	die "Invalid ELF header: EI_CLASS != ELFCLASS32";
}

my $objcopy;

if ($elf[2] == 1) {
	dbg("Detected LE binary\n");
	$objcopy = "mipsel-linux-objcopy";
	@elf = unpack("NC12vvVV", $hdr);
} elsif ($elf[2] == 2) {
	dbg("Detected BE binary\n");
	$objcopy = "mips-linux-objcopy";
	@elf = unpack("NC12nnNN", $hdr);
} else {
	die "Invalid ELF header: EI_DATA != ELFDATA2LSB or ELFDATA2MSB";
}

if ($elf[14] != 8) {
	die "Invalid ELF header: e_machine != EM_MIPS";
}

dbg(sprintf("ELF magic: %08x\n", $elf[0]));
dbg(sprintf("Program entry address: %08x\n", $elf[16]));

unlink($outfile);
run("$objcopy -S -O $outfmt ".
	"--remove-section=.reginfo --remove-section=.mdebug ".
	"--remove-section=/.comment --remove-section=.note ".
	"$infile $outfile");

dbg("$outfmt file written to $outfile\n");

unlink("elf2bin.tmp");
unlink("elf2bin.tmp.gz");

exit 0;
