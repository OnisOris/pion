import code2flow
code2flow.code2flow(['./pion/spion.py', 
                     './pion/pion.py', 
                     './pion/pio.py', 
                     './pion/apion.py', 
                     './pion/functions.py', 
                     './pion/simulator.py'], 
                     './img/graph.png')

code2flow.code2flow(['./pion/pion.py', 
                     './pion/pio.py',
                     './pion/functions.py'], 
                     './img/graph_pion.png')
code2flow.code2flow(['./pion/pion.py', 
                     './pion/pio.py',
                     './pion/functions.py',
                     './pion/apion.py'], 
                     './img/graph_apion.png')

code2flow.code2flow(['./pion/spion.py', 
                     './pion/pio.py'], 
                    './img/graph_spion.png')

code2flow.code2flow(['./pion/simulator.py'], 
                    './img/graph_simulator.png')

code2flow.code2flow(['./pion/pio.py'], 
                    './img/graph_pio.png')
