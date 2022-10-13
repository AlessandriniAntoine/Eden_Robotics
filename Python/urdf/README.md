# URDF

Once the Onshape model is created, having followed the rules stated in the [hardware_folder](../Hardware), you can export it to an urdf format.

As explain on the librairy documentation, you should create onshape API key. It is recommended to store them on your bashrc or zshrc because the secret key will no longer be shown.

```console
// Obtained at https://dev-portal.onshape.com/keys
export ONSHAPE_API=https://cad.onshape.com
export ONSHAPE_ACCESS_KEY=Your_Access_Key
export ONSHAPE_SECRET_KEY=Your_Secret_Key
```

Then, you should create a folder where you want your urdf file to be construct and write a config.json file:

```console
~ $ mkdir -p robot_urdf/config.json
```

the config file must contain at least the following fields (see [config_file](./config.json) for this robot):

```console
{
    "documentId": "document-id",
    "assemblyName": "onshape assembly",
    "outputFormat": "urdf"
}
```

The document-id is the number (below XXXXXXXXX) you can find in Onshape URL:

```console
https://cad.onshape.com/documents/XXXXXXXXX/w/YYYYYYYY/e/ZZZZZZZZ
```

Once this is done, if you properly installed and setup your API key, just run:

```console
~ $ onshape-to-robot robot_urdf
```

If everything goes fine, an urdf file should have been created in the folder, the stl files have been downloaded and part file as well

## Useful Links

- [onshape-to-robot_documentation](https://onshape-to-robot.readthedocs.io/en/latest/)
- [urdf_tutorials](http://wiki.ros.org/urdf/Tutorials)
