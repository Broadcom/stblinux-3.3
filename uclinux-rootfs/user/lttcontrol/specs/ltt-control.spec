#
# Spec file for ltt-control
#
Summary: LTT Control
Name: ltt-control
Version: 0.45
Release: 11102007
License: GPL
Group: Applications/Development
Source: http://ltt.polymtl.ca/lttng/ltt-control-%{version}-%{release}.tar.gz
URL: http://ltt.polymtl.ca
Packager: Martin Bisson <bissonm@discreet.com>
BuildRoot: %{_tmppath}/%{name}-%{version}-%{release}-root-%(%{__id_u} -n)

# Where do we install the libs
%ifarch x86_64 ppc64 ppc64iseries ia64 
%define libdir /usr/lib64
%else
%define libdir /usr/lib
%endif


%description

ltt-control is the part of the Linux Trace Toolkit Next Generation
that allows a machine to be traced.  It holds the daemon with which
the kernel will communicate and the application that starts and
terminate tracing.

%prep
%setup -q -n ltt-control-%{version}-%{release}

%build
./configure --prefix=/usr --libdir=%{libdir}
make

%install
rm -rf $RPM_BUILD_ROOT
make DESTDIR=$RPM_BUILD_ROOT install

%post
echo "Running ldconfig (might take a while)"
ldconfig

%postun
echo "Running ldconfig (might take a while)"
ldconfig

%files
%{libdir}/liblttctl.so.0.0.0
%{libdir}/liblttctl.so.0
%{libdir}/liblttctl.so
%{libdir}/liblttctl.la
%{libdir}/liblttctl.a
/usr/bin/lttctl
/usr/bin/lttd
/usr/include/liblttctl
/usr/include/liblttctl/lttctl.h
