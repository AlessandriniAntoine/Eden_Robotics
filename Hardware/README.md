# Hardware

You can find the model here : [Onshape_model]([aa](https://cad.onshape.com/documents/8404446994fd5ae1242a5d56/w/0f8ef10f6026ba3e07bbdb54/e/7486e3cb0086b1a441d11061?renderMode=0&uiState=634821da4cf1a846282decef)). You need an onshape acount to copy and modify it.

See the assembly instructions [here](./Assembly_Plan.pdf)

## Design Rules

Pour garder un environnement de travail propre, les points suivants doivent être respectés :

- One part studio by part
- Name all part with specific name
- One subassembly by articulation (with fixed join)
- One main assembly containing the motor links
- Assign material to each part
- motor links in the main assembly need be named **dof_<link_name>**
- A relation parent/child need to be respected for every link.

### Parent Child

In Onshape, like all CAO software, there is a parent/child relation in every link. You really need to pay attention to this, it can be source of many errors if you want to export your model.

When you create a link, fixed or motorized, the first element you click on is the child. The second one is the father. You can then check the parent and the child as shown is the following image.

<div style="text-align: center;">
<figure>
    <img src="images/Parent_Child.png" alt=""width="500px">
    <figcaption>Figure 1:Parent Child relation Onshape</figcaption>
</figure>
</div>

## Gmesh

This folder contains a try to create a tool to create easy stl shape from picture or drawing. However, it is still in progress.
