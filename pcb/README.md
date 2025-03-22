# G-5500 Hamlib Adaptor - PCB

See BOM.md for parts list.

## dev-r1 (2025-03-08)

This board revision was developed as a low-effort get-something-on-the-desk, as such it has several intentional limitations.

### DIN Connector Patching

NB some pins include a 15K resistor to solve the leakage current issue.

```
J5  =  J6
1 ----- 1
2 -15K- 2
3 -15K- 3
4 -15K- 4
5 -15K- 5
6 --\-- 7
7 --\-- 8
8 --/-- 6

```

## (future revision)

* Design for enclosure
* * e.g. Hammond 1455K1201BU, PCB: 120mm x 75mm, 32.3mm internal height incl PCB width.

* Remove input polarity protection diode (overkill)

* Use switch-mode DC-DC
* * e.g. TSR1-2450 with appropriate EMI filtering to keep nearby HF receivers happy.

* Use discrete open-collector/drain devices for G-5500 relay control lines to reduce leakage current.

* Remove SWD & RUN connectors on PCB.
