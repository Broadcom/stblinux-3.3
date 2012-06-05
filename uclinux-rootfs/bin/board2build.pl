#!/usr/bin/perl -w

use strict;

my $board = $ARGV[0];
my ($chip, $rev, $combo) = ();

# trivial substitutions (no rev change)
#   chip_family => [ product_id_list ]

my %cheatsheet = (
	"7405" => [ "7406" ],
	"7325" => [ "7324" ],
	"7401" => [ "7402", "7451" ],
	"7403" => [ "7404", "7452" ],
	"7125" => [ "7019", "7025", "7116", "7117", "7119" ],
	"7231" => [ "7229", "7230" ],
	"7340" => [ "7341", "7350", "7351" ],
	"7342" => [ "7352" ],
	"7344" => [ "7354", "7418" ],
	"7346" => [ "7356" ],
	"7358" => [ "7301", "7218", "7357" ],
	"7360" => [ "7302", "7359", "7361" ],
	"7420" => [ "3320", "7409", "7410" ],
	"7425" => [ "3322", "3325", "7421", "7422", "7424" ],
	"7429" => [ "7241", "7242", "7428" ],
	"7435" => [ "7434", "7432", "7431", "3335" ],
	"7468" => [ "7208" ],
	"7550" => [ "7530", "7540", "7560", "7572", "7580" ],
	"7552" => [ "7023", "7531", "7542", "7551", "7573", "7574",
		    "7582", "7592" ],
	"7584" => [ "7585", "7583", "7576", "7586" ],
);

if(defined($ARGV[1])) {
	# grudgingly accept: board2build.pl BOARD REV
	$board .= $ARGV[1];
}

if(! defined($board)) {
	print "usage: board2build.pl <boardname>\n";
	print "\n";
	print "examples:\n";
	print "  board2build.pl 97456d0\n";
	print "  board2build.pl 97466b0\n";
	print "  board2build.pl 97420a0\n";
	exit 0;
}

$board =~ tr/A-Z/a-z/;

$board =~ s/7420dvr2/7420/;
$board =~ s/7420dbs/7420/;
$board =~ s/7420cb([^0-9])/7420$1/;
$board =~ s/7420c([^0-9])/7420$1/;
$board =~ s/7410c([^0-9])/7410$1/;

if($board =~ m/^9?([0-9]{4,5})([a-z])([0-9])$/) {
	$chip = $1;
	$rev = $2."0";
} elsif($board =~ m/^9?([0-9]{4,5})$/) {
	$chip = $1;
	$rev = "a0";
} else {
	print "Invalid format: $board\n";
	exit 1;
}

$combo = $chip.$rev;

# old DOCSIS board variants

if($board =~ m/^97455/) {
	$chip = "7401";
} elsif($board =~ m/^97456/) {
	$chip = "7400";
} elsif($board =~ m/^97458/) {
	$chip = "7403";
} elsif($board =~ m/^97459/) {
	$chip = "7405";
}

# weird mappings involving rev differences

if($chip eq "3549" || $chip eq "3556") {
	$chip = "3548";
	$rev = "b0";
} elsif($combo eq "7413a0" || $combo eq "7414a0") {
	$chip = "7405";
	$rev = "b0";
} elsif($combo eq "7413b0" || $combo eq "7414b0") {
	$chip = "7405";
	$rev = "d0";
} elsif($chip eq "7466") {
	$chip = "7405";
	$rev = "b0";
} elsif($combo eq "7205a0" || $combo eq "7206a0") {
	$chip = "7405";
	$rev = "b0";
} elsif($combo eq "7205b0" || $combo eq "7206b0") {
	$chip = "7405";
	$rev = "d0";
} elsif($chip eq "7213" || $chip eq "7214") {
	$chip = "7405";
	$rev = "d0";
} elsif($chip eq "7336") {
	$chip = "7335";
	$rev = "b0";
}

# easier stuff

foreach my $famid (keys(%cheatsheet)) {
	my $prodlist = $cheatsheet{$famid};
	foreach my $member (@$prodlist) {
		if($chip eq $member) {
			$chip = $famid;
		}
	}
}

# backward-compatible revs

if($chip eq "7125" && ($rev eq "d0" || $rev eq "e0")) {
	$rev = "c0";
}

if($chip eq "7325" && ($rev eq "c0" || $rev eq "d0")) {
	$rev = "b0";
}

if($chip eq "7420" && $rev eq "d0") {
	$rev = "c0";
}

print "${chip}${rev}\n";

exit 0;
