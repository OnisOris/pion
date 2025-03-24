import code2flow

code2flow.code2flow(
    [
        "./pion/spion.py",
        "./pion/pion.py",
        "./pion/pio.py",
        "./pion/apion.py",
        "./pion/functions.py",
        "./pion/simulator.py",
    ],
    "../docs/img/graph.png",
)

code2flow.code2flow(
    ["./pion/pion.py", "./pion/pio.py", "./pion/functions.py"],
    "docs/img/graph_pion.png",
)
code2flow.code2flow(
    [
        "./pion/pion.py",
        "./pion/pio.py",
        "./pion/functions.py",
        "./pion/apion.py",
    ],
    "docs/img/graph_apion.png",
)

code2flow.code2flow(
    ["./pion/spion.py", "./pion/pio.py"], "docs/img/graph_spion.png"
)

code2flow.code2flow(["./pion/simulator.py"], "docs/img/graph_simulator.png")

code2flow.code2flow(["./pion/pio.py"], "docs/img/graph_pio.png")

code2flow.code2flow(["./pion/functions.py"], "../docs/img/graph_functions.png")
