ACLOCAL_AMFLAGS = -I m4
AM_CPPFLAGS = -I$(top_srcdir)/include

bin_PROGRAMS = pirctrl pirfilt pirdump

noinst_PROGRAMS = pirtest testkin

pirdump_SOURCES = src/pirdump.c
pirdump_LDADD = -lsns -lach  -lreflex -lamino -lblas -llapack

pirctrl_SOURCES = src/pirctrl.c src/setmode.c src/sdh.c src/ctrl.c
pirctrl_LDADD = -lsns -lach  -lreflex -lamino -lblas -llapack

lib_LTLIBRARIES = libpiranha.la

libpiranha_la_SOURCES =                  \
	tf.f                             \
	src/lwa4.c                       \
	src/kinematics.cpp

# check_can_SOURCES = src/check-can.c
# check_can_LDADD = -lsns  -lamino -lsocanmatic -lblas -llapack


pirtest_SOURCES = src/pirtest.c
pirtest_LDADD = -lsns  -lamino -lsocanmatic -lblas -llapack -lreflex libpiranha.la


testkin_SOURCES = src/testkin.c
testkin_LDADD = -lsns  -lamino -lsocanmatic -lblas -llapack -lreflex libpiranha.la


pirfilt_SOURCES = src/pirfilt.c
pirfilt_LDADD = -lsns -lach -lamino -lblas -llapack libpiranha.la -lreflex libpiranha.la

kinematics.f: $(top_srcdir)/src/kinematics.mac
	maxima --very-quiet -b $(top_srcdir)/src/kinematics.mac
