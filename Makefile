
ARCHIVE="slime-20110729.tar.bz2"
ARCHIVE_URL="http://code.in.tum.de/files/$(ARCHIVE)"

all: .slime

.slime: Makefile
	make clean
	wget $(ARCHIVE_URL)
	tar xvjf $(ARCHIVE)
	touch slime/rospack_nosubdirs
	touch .slime

clean:
	rm -rf slime .slime $(ARCHIVE)

