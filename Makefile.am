ACLOCAL_AMFLAGS = -I m4
AM_CPPFLAGS = -I$(top_srcdir)/include

bin_PROGRAMS = pirctrl

pirctrl_SOURCES = src/pirctrl.c kinematics.f
pirctrl_LDADD = -lsns -lach

kinematics.f: $(top_srcdir)/src/kinematics.mac
	maxima --very-quiet -b $(top_srcdir)/src/kinematics.mac
