#!/usr/bin/perl -w

# generate a list of environment variables that is not in the whitelist
# whitelist is provided on the command line, e.g.
# perl -w config/cleanenv.pl PATH SHELL HOME USER

my %ex = ( );
foreach my $x (@ARGV)
{
	$ex{$x} = 1;
}

foreach my $x (keys(%ENV))
{
	if(! defined($ex{$x}))
	{
		print "$x\n";
	}
}

exit 0;
