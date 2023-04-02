tests:
	set -e; \
	for i in *Test/Makefile; do \
		echo '==== Making:' $$(dirname $$i); \
		$(MAKE) -C $$(dirname $$i) clean; \
		$(MAKE) -C $$(dirname $$i); \
	done

runtests:
	set -e; \
	for i in *Test/Makefile; do \
		echo '==== Running:' $$(dirname $$i); \
		$(MAKE) -s -C $$(dirname $$i) clean; \
		$(MAKE) -s -C $$(dirname $$i) all; \
		$(MAKE) -C $$(dirname $$i) run; \
	done

clean:
	set -e; \
	for i in *Test/Makefile; do \
		echo '==== Cleaning:' $$(dirname $$i); \
		$(MAKE) -C $$(dirname $$i) clean; \
	done