Scenario data.
`*.uxsim_scenario` are new file formats that can be loaded by `World.load_scenario(file_name)`.
`*.csv` are old format, but they still work.

- `sfnetwork.uxsim_scenario`: The famous Sioux Falls network with 30000 vehicles. Recommended setting for `World` is `tmax=7200` and `deltan>=5`.
- `sfnetwork.uxsim_scenario`: Abstract road network in Chicago metropolitan area with about 1 million vehicles. Recommended setting for `World` is `tmax=10000` and `deltan>=30`.

Source:
`siouxfalls_*` and `chicago_sketch*` are derived from Transportation Networks for Research https://github.com/bstabler/TransportationNetworks by Transportation Networks for Research Core Team.
`tokyo_arterial_*` are derived from OpenStreetMap https://www.openstreetmap.org/.