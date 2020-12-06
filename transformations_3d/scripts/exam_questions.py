#!/usr/bin/env python

from IPython.display import display, Markdown
import random

class Exam_eucl_space:
    def __init__(self):
        self.questions_1 = [
            ["How to compute the distance between two rotation matrices?",
             "Comment calculer la distance entre deux matrices de rotation ?"],
            ["What is a gimbal lock?",
             "Qu'est-ce qu'un blocage de cardan ?"],
            ["What is the identity element of the Special orthogonal group?",
             "Quel est l'élément neutre du groupe orthogonal spécial ?"],
            ["What is the identity element of the Special Euclidean group?",
             "Quel est l'élément neutre du groupe euclidien spécial ?"],
            ["What is the operator associated with a rotation matrix that defines the Special orthogonal group?",
             "Quelle opération associée à une matrice de rotation permet de définir le groupe orthogonal spécial ?"],
            ["What is the operator associated with a rotation matrix that defines the Special Euclidean group?",
             "Quelle opération associée à une matrice de rotation permet de définir le groupe euclidien spécial ?"],
            ["What is the link between a rotation matrix and a frame of reference?",
             "Quel est le lien entre une matrice de rotation et un référentiel ?"],
            ["What is the difference between an intrinsic and an extrinsic rotation?",
             "Quel est la différence entre une rotation intrinsèque et extrinsèque ?"],
            ["Give an advantage of rotation matrix over other rotation representation.",
             "Donnez un avantage des matrices de rotation par rapport aux autres représentations des rotations."],
            ["Give a disadvantage of rotation matrix over other rotation representation.",
             "Donnez un désavantage des matrices de rotation par rapport aux autres représentations des rotations."],
            ["Give an advantage of quaternion over other rotation representation.",
             "Donnez un avantage des quaternions par rapport aux autres représentations des rotations."],            
            ["Give a disadvantage of quaternion over other rotation representations.",
             "Donnez un désavantage des quaternions par rapport aux autres représentations des rotations."],
            ["What is the link between a rotation matrix and a reflexion matrix?",
             "Quel est le lien entre une matrice de rotation et une matrice de réflexion ?"],
            ["What are the four parameters of a quaternion producing no rotation?",
             "Quels sont les quatre paramètres d'un quaternion ne produisant aucune rotation ?"],
            
        ]
        self.questions_2 = [
            ["What is or are the constraints defining an axis-angle representation?",
             "Quelles sont la/les contraintes définissant la représentation axe-angle ?"],
            ["What is or are the constraints defining a rotation matrix?",
             "Quelles sont la/les contraintes définissant une matrice de rotation ?"],
            ["What is or are the constraints defining a quaternion?",
             "Quelles sont la/les contraintes définissant un quaternion ?"],
            ["Give an advantage of Euler angles over other rotation representations.",
             "Donnez un avantage de la représentation des angles d'Euler par rapport aux autres représentations des rotations."], 
            ["Give a disadvantage of Euler angles over other rotation representations.",
             "Donnez un désavantage de la représentation des angles d'Euler par rapport aux autres représentations des rotations."],
            ["Give an advantage of axis-angle representation over other rotation representations.",
             "Donnez un avantage de la représentation axe-angle par rapport aux autres représentations des rotations."],
            ["Give a disadvantage of axis-angle representation over other rotation representations.",
             "Donnez un désavantage de la représentation axe-angle par rapport aux autres représentations des rotations."],
            ["What is a principal rotation?",
             "Qu'est-ce qu'une rotation de base ?"],
            ["Explain what are nautical angles.",
             "Expliquez qu'est que les angles nautiques."],
            ["What is the difference between proper Euler and Tait–Bryan angles?",
             "Quelle est la différence entre les angles originaux d'Euler et les angles Tait-Bryan ?"],
            [r"In the axis-angle representation, what happen to the axis when $\theta=0$?",
             r"Dans la représentation axe-angle, qu'arrive-t-il à l'axe lorsque $\theta=0$ ?"],
            [r"In the axis-angle representation, what happen to the axis when $\theta=\pi$?",
             r"Dans la représentation axe-angle, qu'arrive-t-il à l'axe lorsque $\theta=\pi$ ?"],
            ["What is defining a frame of reference?",
             "Qu'est-ce qui définit un référentiel ?"],
            ["Which homogeneous coordinates cannot be expressed in cartesian coordinates?",
             "Quelles coordonnées homogènes ne peuvent pas être exprimées en coordonnées cartésiennes ?"],
            
        ]
        
        self.questions_3 = [
            ["What is the minimal number of parameters required to represent a 3D rotation?",
             "Quel est le nombre minimal de paramètres requis pour représenter une rotation ?"],
            ["What is the result of the cross product?",
             "Quel est le résultat d'un produit vectoriel ?"],
            ["What is the operation connecting the cross product and the dot product?",
             "Quel opération relie le produit vectoriel et le produit scalaire ?"],
            ["How many different shear matrices are possible in 3D?",
             "Combien de matrices de transvection sont possibles en 3D ?"],
            [r"Express the following quaternion $1 + 2i + 3j + 4k$ as a vector.",
             r"Exprimez le quaternion $1 + 2i + 3j + 4k$ en vecteur."],
            [r"What is the difference between $\SO{3}$ and $\SE{3}$?",
             r"Quelle est la différence entre $\SO{3}$ and $\SE{3}$ ?"],
            ["What format would have a point cloud that needs to be transformed by a matrix in $\SE{3}$ using a single matrix multiplication?",
             "Quel format aurait un nuage de point devant être transformé par une matrice faisant partie de $\SE{3}$ en utilisant une seule multiplication matricielle ?"],
            ["What format would have a point cloud that needs to be transformed by a matrix in $\SO{3}$ using a single matrix multiplication?",
             "Quel format aurait un nuage de point devant être transformé par un matrice faisant partie de $\SO{3}$ en utilisant une seule multiplication matricielle ?"],
        ]
        pass
        
    def print_questions(self, questions):
        str = ""
        for q in questions:
            str += "1. EN: " + q[0] + '\n\n'
            str += "   FR: _" + q[1] + '_\n'
        display(Markdown(str))
        pass
    
    def show_all(self):
        all_questions = self.questions_1 + self.questions_2 + self.questions_3
        random.shuffle(all_questions)
        self.print_questions(all_questions)
        pass
    
    def random_sampling(self):        
        display(Markdown("## Question 1 (10 points):"))
        sampling = random.sample(self.questions_1, k=1)
        self.print_questions(sampling)
        
        display(Markdown("## Question 2 (10 points):"))
        sampling = random.sample(self.questions_2, k=1)
        self.print_questions(sampling)
        
        display(Markdown("## Question 3 (5 points):"))
        sampling = random.sample(self.questions_3, k=1)
        self.print_questions(sampling)
        pass
       