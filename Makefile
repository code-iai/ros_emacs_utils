
ARCHIVE="slime-20110729.tar.bz2"
ARCHIVE_URL="http://code.in.tum.de/files/$(ARCHIVE)"

all: .slime .swank

.slime: Makefile
	make clean
	wget $(ARCHIVE_URL)
	tar xvjf $(ARCHIVE)
	touch slime/rospack_nosubdirs
	touch .slime

.swank: 
	[ -f $(HOME)/.swank.lisp ] || cp swank.lisp $(HOME)/.swank.lisp
	touch .swank
	
clean:
	rm -rf slime .slime .swank $(ARCHIVE)

