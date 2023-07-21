Working on how to upload STM32Cube files to github.
Not as easy as hoped, but ...

20230718 - Plan is to create new empty STM32Cube project to work via github.
 Once confident, whole project can be migrated.
Copying or renaming an 'Eclipse' project is not trivial.

21st July 2023 - Working on CAN bus interface. Looking good, got Tx working, and Rx on loopback.
Lot to learn about CAN, having not used it to date. Looks very good for the purpose.
Keen on 'filters', CAN hardware includes bank of 14 filters which can be used to pre-sort messages into numbered category and to reject messages of no interest. Opens the way for broad/narrowcasting. 8 byte data length max, but multi-packet messagig not difficult.
