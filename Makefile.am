ACLOCAL_AMFLAGS = -I m4
AM_CPPFLAGS = -I$(top_srcdir)/include

bin_PROGRAMS = pirctrl pirfilt

noinst_PROGRAMS = check-can

pirctrl_SOURCES = src/pirctrl.c
pirctrl_LDADD = -lsns -lach  -lreflex -lamino


check_can_SOURCES = src/check-can.c
check_can_LDADD = -lsns  -lamino -lsocanmatic

pirfilt_SOURCES = src/pirfilt.c kinematics.f
pirfilt_LDADD = -lsns -lach -lamino

kinematics.f: $(top_srcdir)/src/kinematics.mac
	maxima --very-quiet -b $(top_srcdir)/src/kinematics.mac
