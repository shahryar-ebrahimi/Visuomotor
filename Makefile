default: run

kill: rob # Kill the robot, reasonably gently
	@if pgrep -x "robot" > /dev/null ; then echo "\n\n-- Killing with Python --\n\n" ;python3 -c "import robot.interface as robot; robot.start_shm(); robot.unload();" ; else echo "\n\n-- No robot process running --\n\n"; fi

doc: documentation.html
	xdg-open documentation.html

documentation.html: readme.md fonts/github-pandoc.css
	pandoc -f markdown -t html readme.md -s -c fonts/github-pandoc.css -o documentation.html

run: rob
	python3 run.py 

dummy:
	python3 run.py dummy

rob:
	make -C robot

clean:
	rm -f *.pyc
	rm -f *~
	make -C robot clean



