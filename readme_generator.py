from readmegen import ReadmeGenerator
import os

if __name__ == "__main__":
    dirs = ["img", "description"]
    for di in dirs:
        if not (os.path.exists(di) and os.path.isdir(di)):
            os.mkdir(di)
    lists = [[['./pion/spion.py',
                         './pion/pion.py',
                         './pion/pio.py',
                         './pion/apion.py',
                         './pion/functions.py',
                         './pion/simulator.py'],
                        './img/graph.png'],
             [['./pion/pion.py',
                         './pion/pio.py',
                         './pion/functions.py'],
                        './img/pion.png'],
             [['./pion/pion.py',
                         './pion/pio.py',
                         './pion/functions.py',
                         './pion/apion.py'],
                        './img/apion.png'],
             [['./pion/spion.py',
                         './pion/pio.py'],
                        './img/spion.png'],
             [['./pion/simulator.py'],
                        './img/simulator.png'],
             [['./pion/pio.py'],
                        './img/pio.png'],
             [['./pion/functions.py'],
                        './img/functions.png']]
    rgen = ReadmeGenerator(lists)

    rgen.generate('./pion/')
             
