import matplotlib.pyplot as plt


def der(x):
    d = []
    for i in range(3, len(x) - 1):
        d.append(x[i] - x[i - 1])
    return d


with open("pos.test") as file:
    lines = file.readlines()

    x = []
    y = []

    for lin in lines:

        # if lin.startswith("x:"):
        #
        #    val = float(lin.split()[1])
        #    x.append(val)

        spl = lin.split()

        if len(spl) > 2 and spl[-2] == "x:":
            x.append(float(spl[-1]))

            t = spl[2]
            t = t.replace("[", "")
            t = t.replace("]", "")
            y.append(float(t))
    # plt.plot(y[140:190], x[140:190])  #

    # plt.plot(der(x))

with open("t2.test") as file:
    lines = file.readlines()

    x = []

    for lin in lines:
        if lin.startswith("x:"):

            val = float(lin.split()[1])
            x.append(val)
    # plt.plot(x[30:80])
    plt.plot(der(x))

plt.show()
