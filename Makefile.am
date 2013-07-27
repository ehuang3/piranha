ACLOCAL_AMFLAGS = -I m4
AM_CPPFLAGS = -I$(top_srcdir)/include

bin_PROGRAMS = pirctrl pirfilt

noinst_PROGRAMS = pirtest

pirctrl_SOURCES = src/pirctrl.c
pirctrl_LDADD = -lsns -lach  -lreflex -lamino -lblas -llapack

lib_LTLIBRARIES = libpiranha.la

libpiranha_la_SOURCES =                  \
	kinematics.f                     \
	tf.f                             \
	src/lwa4.c

# check_can_SOURCES = src/check-can.c
# check_can_LDADD = -lsns  -lamino -lsocanmatic -lblas -llapack


pirtest_SOURCES = src/pirtest.c
pirtest_LDADD = -lsns  -lamino -lsocanmatic -lblas -llapack -lreflex libpiranha.la

pirfilt_SOURCES = src/pirfilt.c
pirfilt_LDADD = -lsns -lach -lamino -lblas -llapack libpiranha.la -lreflex

kinematics.f: $(top_srcdir)/src/kinematics.mac
	maxima --very-quiet -b $(top_srcdir)/src/kinematics.mac
