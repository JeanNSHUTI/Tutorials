# Administration

To add new members to the organization and configure their permissions, we need an
administrator. Each year an administrator is designed who preferably has experience with
Git and GitHub in order to help his peers and resolve any Git problems they may face.

> This chapter is destined for the person who administrates the GitHub organization

The administrators for past years were:

- **2018** [@azerupi](https://github.com/azerupi) (Mathieu David)
- **2017** [@charlesvdv](https://github.com/charlesvdv) (Charles Vandevoorde)

You can contact them (most recent first preferably) if you are the chosen administrator for your team.
They will add you with the correct privileges.

## Creating a new repository

As administrator, your task is to setup GitHub and the repositories so that everyone can work on the 
project. The first thing you want to do is create a new repository for your year.

We do however recommend to base it off off the repository from previous year instead of starting from scratch.
As administrator, head over to the [organization's page](https://github.com/Ecam-Eurobot) to create a new repository.

![org](img/organization/org.png)

Fill in the name with `Eurobot-xxxx` where `xxxx` represents the year. You can add a description if you want. Leave all
the rest blank because we are going to push the code of previous year in the newly created repository.

![new repository](img/organization/new-repo.png)

You will land on the following page

![new repository created](img/organization/new-repo2.png)

Let's push the repository from last year into the newly created repository.
On the command line, clone last years repository. In this case, I will clone [Eurobot-2018](https://github.com/Ecam-Eurobot/Eurobot-2018)

```
git clone https://github.com/Ecam-Eurobot/Eurobot-2018.git
cd Eurobot-2018
```

Now you need to add the newly created repository as a remote. And verify that it was added correctly.

```
git remote add next-year https://github.com/Ecam-Eurobot/Eurobot-2019.git
git remote -v
```

Push the repository into the newly created repository

```
git push -u next-year master
```

If we refresh the GitHub page for the newly created repository, we can see something similar to below.
We can see (in the top left) that we are in the new repository and if we look at the `README` at the 
bottom we can see that it contains the files from previous year. We can now start to work in the new 
repository without affecting the old one.

![Repository pushed](img/organization/new-repo3.png)

> **Note:**  
> Now that the new repository is created, you can remove the one from last year that you cloned.
> You will not need it anymore.