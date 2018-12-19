CC=$(CROSS_COMPILE)gcc
file = tm1638_test
ALL:
	$(CC) -o $(file) $(file).c

install:
	scp $(file) $(INSTALLDIR)

clean:
	rm -rf $(file)

