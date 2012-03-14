Summary:		Linux Hotplug Scripts
Name:			hotplug
Version:		2004_04_01
Release:		1
Group:			Applications/System
License:		GPL
Url:			http://linux-hotplug.sourceforge.net/
Source0:		%{name}-%{version}.tar.gz
BuildRoot:		%{_tmppath}/%{name}-root
BuildArchitectures:	noarch
Prereq:			/sbin/chkconfig
Requires:		%{name}-base >= %{version}

%package base
Summary:		Base /sbin/hotplug multiplexer script
Group:			Applications/System
License:		GPL

%description
This package contains the scripts necessary for hotplug Linux support.

%description base
This package contains the base /sbin/hotplug script that allows other programs
to be executed when /sbin/hotplug is called


%prep
%setup -q

#%build
# Build any compiled programs in the source tree.
#make all CFLAGS="%optflags"

%install
make prefix=${RPM_BUILD_ROOT} install

%clean
rm -rf $RPM_BUILD_ROOT

# --
# The core package contains the directory structure, the main hotplug
# program, and all the basic infrastructure to make the hotplug process
# work on the machine.

%files
%defattr(-,root,root)
/etc/init.d/*
/etc/hotplug/*
/etc/hotplug.d/*
%dir /var/run/usb
%dir /var/log/hotplug
%{_mandir}/*
%doc README ChangeLog

%files base
%defattr(-,root,root)
/sbin/hotplug
%dir /etc/hotplug.d

%post
/sbin/chkconfig --add hotplug
# Uncomment this out if we find that we need to restart the system when
# we have loaded a new copy of the package.
#if test -r /var/lock/subsys/hotplug ; then
#	/etc/init.d/hotplug restart >&2
#fi


%preun
if [ "$1" = 0 ] ; then
	/etc/init.d/hotplug stop >&2
	/sbin/chkconfig --del hotplug
fi


%changelog
* Tue Mar 26 2002 Greg Kroah-Hartman <greg@kroah.com>
- moved the fxload program out of the core hotplug package

* Mon Jun 11 2001 Steve Williams <steve@icarus.com>
- The install process is now in the comon makefile, so that non-
  rpm installs can work. This spec file thus invokes that install.

* Fri Jun 8 2001 Steve Williams <steve@icarus.com>
- added the /var/run/usb directory to spec file

* Tue Apr 24 2001 Greg Kroah-Hartman <greg@kroah.com>
- added the hotplug.8 manpage written by Fumitoshi UKAI <ukai@debian.or.jp>

* Fri Mar 2 2001 Greg Kroah-Hartman <greg@kroah.com>
- tweaked the post and preun sections to fix problem of hotplug
  not starting automatically when the package is upgraded.

* Wed Feb 28 2001 Greg Kroah-Hartman <greg@kroah.com>
- 2001_02_28 release

* Wed Feb 14 2001 Greg Kroah-Hartman <greg@kroah.com>
- 2001_02_14 release

* Wed Jan 17 2001 Greg Kroah-Hartman <greg@kroah.com>
- changed specfile based on Chmouel Boudjnah's <chmouel@mandrakesoft.com> comments.

* Tue Jan 16 2001 Greg Kroah-Hartman <greg@kroah.com>
- tweaked the file locations due to the change in the tarball structure.
- 2001_01_16 release

* Mon Jan 15 2001 Greg Kroah-Hartman <greg@kroah.com>
- First cut at a spec file for the hotplug scripts.
- added patch to usb.rc to allow chkconfig to install and remove it.

