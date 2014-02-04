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
	pir-frame.c                      \
	src/kinematics.cpp

# check_can_SOURCES = src/check-can.c
# check_can_LDADD = -lsns  -lamino -lsocanmatic -lblas -llapack


pirtest_SOURCES = src/pirtest.c
pirtest_LDADD = -lsns  -lamino -lsocanmatic -lblas -llapack -lreflex libpiranha.la


testkin_SOURCES = src/testkin.c
testkin_LDADD = -lsns  -lamino -lsocanmatic -lblas -llapack -lreflex libpiranha.la


pirfilt_SOURCES = src/pirfilt.c
pirfilt_LDADD = -lsns -lach -lamino -lblas -llapack libpiranha.la -lreflex libpiranha.la


bin_PROGRAMS += pir-cal
pir_cal_SOURCES = src/pir-cal.c
pir_cal_LDADD = -lsns -lach -lamino -lblas -llapack libpiranha.la -lreflex libpiranha.la


bin_PROGRAMS += pir-dump
pir_dump_SOURCES = src/pir-dump.c
pir_dump_LDADD = -lsns -lach -lamino -lblas -llapack libpiranha.la -lreflex libpiranha.la

BUILT_SOURCES = pir-frame.h pir-frame.c

pir-frame.c: lisp/kinematics.lisp
	sbcl --script lisp/kinematics.lisp

pir-frame.dot: pir-frame.c
pir-frame.h: pir-frame.c

pir-frame.pdf: pir-frame.dot
	dot pir-frame.dot -Tpdf > pir-frame.pdf

kinematics.f: $(top_srcdir)/src/kinematics.mac
	maxima --very-quiet -b $(top_srcdir)/src/kinematics.mac


noinst_PROGRAMS += iros2014
iros2014_SOURCES = src/experiments/iros2014.c
iros2014_LDADD = libpiranha.la -lreflex -lamino -llapack -lblas

clean-local:
	rm -f pir-frame.h pir-frame.c pir-frame.dot
