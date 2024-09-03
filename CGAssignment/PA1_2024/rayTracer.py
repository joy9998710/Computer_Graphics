#!/usr/bin/env python3
# -*- coding: utf-8 -*
# sample_python aims to allow seamless integration with lua.
# see examples below

import os
import sys
import pdb  # use pdb.set_trace() for debugging
import code # or use code.interact(local=dict(globals(), **locals()))  for debugging.
import xml.etree.ElementTree as ET
import numpy as np
from PIL import Image 
class Color:
    def __init__(self, R, G, B):
        self.color=np.array([R,G,B]).astype(np.float64)

    # Gamma corrects this color.
    # @param gamma the gamma value to use (2.2 is generally used).
    def gammaCorrect(self, gamma):
        inverseGamma = 1.0 / gamma;
        self.color=np.power(self.color, inverseGamma)

    def toUINT8(self):
        return (np.clip(self.color, 0,1)*255).astype(np.uint8)

class Camera:
    def __init__(self, viewPoint, viewDir, viewUp, projNormal, projDistance, viewWidth, viewHeight):
        self.viewPoint = viewPoint
        self.viewDir = viewDir
        self.viewUp = viewUp
        #-1 * ViewDir
        self.projNormal = projNormal
        self.projDistance = projDistance
        self.viewWidth = viewWidth
        self.viewHeight = viewHeight
        #For future calculation of pixel to #D point mapping
        #Calculating vector that indicates the center of view rectangle
        self.viewCenter = self.viewPoint + self.projDistance * self.viewDir/np.linalg.norm(self.viewDir)
        #Calculating horizontal vector of view rectangle
        self.horizontal = np.cross(self.viewDir, self.viewUp)
        self.horizontal /= np.linalg.norm(self.horizontal)
        #Calculating vertical vector of view rectangle
        self.vertical = np.cross(self.horizontal, self.viewDir)
        self.vertical /= np.linalg.norm(self.vertical)
        #vCalculating lower left corner of view rectangle
        self.lower_left_corner = self.viewCenter - (self.viewWidth * self.horizontal)/2 - (self.viewHeight * self.vertical)/2

    def get_ray(self, x, y, imgWidth, imgHeight):
        u = self.viewWidth * ((x + 0.5) / imgWidth)
        v = self.viewHeight * ((y + 0.5) / imgHeight)

        #Mapping pixel to view rectangle point
        viewRectangle_point = self.lower_left_corner + u*self.horizontal + v*self.vertical
        #Calculating ray direction
        ray_direction = viewRectangle_point - self.viewPoint
        ray_direction /= np.linalg.norm(ray_direction)

        return Ray(self.viewPoint, ray_direction)

class Ray:
    #p is the origin and d is the direction
    def __init__(self, origin, directional_vector):
        self.origin = origin
        self.directional_vector = directional_vector
        #normalize the directional vector
        self.directional_vector /= np.linalg.norm(self.directional_vector)

    def ray_line(self, ray_parameter):
        return self.origin + ray_parameter * self.directional_vector


class Sphere:
    def __init__(self, center, radius, color):
        self.center = center
        self.radius = radius
        self.color = color
    
    def intersect(self, ray, light_position, spheres):
        #Check wheter sphere and intersect or not
        #Using the quadratic formula (discriminant)
        a = np.dot(ray.directional_vector, ray.directional_vector)
        b = 2 * np.dot(ray.directional_vector, (ray.origin - self.center))
        c = np.dot((ray.origin - self.center), (ray.origin - self.center)) - self.radius * self.radius

        discriminant = b * b - 4 * a * c

        #Case that ray intersects sphere
        if discriminant > 0:
            #Check if the sphere is on the opposite direction of ray
            t1 = (-b - np.sqrt(discriminant)) / (2.0 * a)
            t2 = (-b + np.sqrt(discriminant)) / (2.0 * a)

            t = None
            if t1 > 0 and t2 > 0:
                t = min(t1,t2)
            elif t1 > 0:
                t = t1
            elif t2 > 0:
                t = t2

           
            intersect_point = ray.ray_line(t)
            normal = intersect_point - self.center
            normal /= np.linalg.norm(normal)

            is_in_shadow = False
            shadow_origin = intersect_point
            shadow_direction = light_position - shadow_origin
            shadow_ray = Ray(shadow_origin, shadow_direction)
                
            for sphere in spheres:
                if sphere is not self and sphere.intersect_shadow(shadow_ray):
                    is_in_shadow = True
                    break
            return (intersect_point, normal, t, is_in_shadow)
        return None

    def intersect_shadow(self, ray):
        a = np.dot(ray.directional_vector, ray.directional_vector)
        b = 2 * np.dot(ray.directional_vector, (ray.origin - self.center))
        c = np.dot((ray.origin - self.center), (ray.origin - self.center)) - self.radius**2

        discriminant = b*b - 4*a*c

        if discriminant >= 0:
            t1 = (-b - np.sqrt(discriminant)) / (2.0 * a)
            t2 = (-b + np.sqrt(discriminant)) / (2.0 * a)

            if t1 > 0 or t2 > 0:
                return True

        return False




class Shade:
    def __init__(self, name, light_position, light_intensity, camera_position, diffuse_color, specular_color, shininess):
        self.name = name
        self.light_position = light_position
        self.light_intensity = light_intensity
        self.camera_position = camera_position
        self.diffuse_color = diffuse_color
        self.specular_color = specular_color
        self.shininess = shininess

    #Diffuse shading (Lambertian)
    def diffuse(self, diffuse_coefficient, hit_point, normal):
        light_vector = self.light_position - hit_point
        light_vector /= np.linalg.norm(light_vector)
        normal /= np.linalg.norm(normal)
        diffuse_intensity = max(0, np.dot(normal, light_vector))
        diffuseR = diffuse_coefficient * diffuse_intensity * self.light_intensity[0] * self.diffuse_color[0]
        diffuseG = diffuse_coefficient * diffuse_intensity * self.light_intensity[1] * self.diffuse_color[1]
        diffuseB = diffuse_coefficient * diffuse_intensity * self.light_intensity[2] * self.diffuse_color[2]

        diffuse_color = np.array([diffuseR, diffuseG, diffuseB])
        
        return diffuse_color

    def phong(self, specular_coefficient, hit_point, normal):
        view_vector = self.camera_position - hit_point
        view_vector /= np.linalg.norm(view_vector)
        light_vector = self.light_position - hit_point
        light_vector /= np.linalg.norm(light_vector)
        h = (light_vector + view_vector) / np.linalg.norm(light_vector + view_vector) #bisector
        specular_intensity = np.power(max(0, np.dot(normal, h)), self.shininess)
        specularR = specular_coefficient *specular_intensity * self.light_intensity[0] * self.specular_color[0]
        specularG = specular_coefficient *specular_intensity * self.light_intensity[1] * self.specular_color[1]
        specularB = specular_coefficient *specular_intensity * self.light_intensity[2] * self.specular_color[2]

        specular_color = np.array([specularR, specularG, specularB])

        return specular_color

    #shading total
    def shading(self,diffuse_coefficient, specular_coefficient, hit_point, normal):
        color = self.diffuse(diffuse_coefficient, hit_point, normal) + self.phong(specular_coefficient, hit_point, normal)
        return Color(color[0], color[1], color[2])



def main():
    tree = ET.parse(sys.argv[1])
    root = tree.getroot()

    # set default values
    viewDir=np.array([0,0,-1]).astype(np.float64)
    viewUp=np.array([0,1,0]).astype(np.float64)
    viewProjNormal=-1*viewDir  # you can safely assume this. (no examples will use shifted perspective camera)
    viewWidth=1.0
    viewHeight=1.0
    projDistance=1.0
    light_intensity=np.array([1,1,1]).astype(np.float64)  # how bright the light is.
    diffuse_color = np.array([0,0,0]).astype(np.float64)
    specular_color = np.array([0,0,0]).astype(np.float64)

    imgSize=np.array(root.findtext('image').split()).astype(np.int32)

    #Setting the None value for classes
    camera = None
    shaders = []
    spheres = []
    light_position = None
    # Create an empty image
    channels=3
    img = np.zeros((imgSize[1], imgSize[0], channels), dtype=np.uint8)
    img[:,:]=0

    for c in root.findall('camera'):
        viewPoint=np.array(c.findtext('viewPoint').split()).astype(np.float64)
        viewDir = np.array(c.findtext('viewDir').split()).astype(np.float64)
        viewUp = np.array(c.findtext('viewUp').split()).astype(np.float64)
        projNormal = np.array(c.findtext('projNormal').split()).astype(np.float64)
        if c.findtext('projDistance') is not None:
            projDistance = float(c.findtext('projDistance'));
        viewWidth = float(c.findtext('viewWidth'))
        viewHeight = float(c.findtext('viewHeight'))

    camera = Camera(viewPoint, viewDir, viewUp, projNormal, projDistance, viewWidth, viewHeight)

    for c in root.findall('light'):
        light_position = np.array(c.findtext('position').split()).astype(np.float64)
        light_intensity = np.array(c.findtext('intensity').split()).astype(np.float64)
    
    for c in root.findall('shader'):
        name = None
        shininess = None
        if c.get('type') == 'Lambertian':
            if c.get('name') == "red":
                name = "red"
                diffuse_color = np.array(c.findtext('diffuseColor').split()).astype(np.float64)
                shininess = 0
            elif c.get('name') == 'blue':
                name = "blue"
                diffuse_color = np.array(c.findtext('diffuseColor').split()).astype(np.float64)
                shininess = 0
            elif c.get('name') == 'green':
                name = "green"
                diffuse_color = np.array(c.findtext('diffuseColor').split()).astype(np.float64)
                shininess = 0
            elif c.get('name') == 'gray':
                name = "gray"
                diffuse_color = np.array(c.findtext('diffuseColor').split()).astype(np.float64)
                shininess = 0
        elif c.get('type') == 'Phong':
            name = c.get('name')
            diffuse_color = np.array(c.findtext('diffuseColor').split()).astype(np.float64)
            specular_color = np.array(c.findtext('specularColor').split()).astype(np.float64)
            shininess = float(c.findtext('exponent'))
        else:
            continue
        shaders.append(Shade(name,light_position, light_intensity, camera.viewPoint, diffuse_color, specular_color, shininess))

    for c in root.findall('surface'):
        if c.get('type') == 'Sphere':
            center = np.array(c.findtext('center').split()).astype(np.float64)
            radius = float(c.findtext('radius'))
            if c.find('shader').get('ref') == 'red':
                color = 'red'
            elif c.find('shader').get('ref') == 'blue':
                color = 'blue'
            elif c.find('shader').get('ref') == 'green':
                color = 'green'
            elif c.find('shader').get('ref') == 'gray':
                color = 'gray'

            spheres.append(Sphere(center,radius,color))

    for y in range(imgSize[1]):
        for x in range(imgSize[0]):
            ray = camera.get_ray(x, y, imgSize[1], imgSize[0])
            name = None
            #Initilize ray param as INF
            ray_parm = 987654321
            point = None
            for sphere in spheres:
                intersect_point = sphere.intersect(ray, light_position, spheres)
                if intersect_point is not None:
                    temp = intersect_point[2]
                    if ray_parm >= temp:
                        ray_parm = temp
                        point = intersect_point
                        name = sphere.color
            if ray_parm != 987654321:
                if point[3]:
                    img[imgSize[1] - y - 1, x] = [0,0,0]
                    continue
                for shade in shaders:
                    if(name == shade.name):
                        color = shade.shading(1.8, 1.8, point[0], point[1])
                        img[imgSize[1] - y-1,x] = color.toUINT8()
                        break
                    else:
                        img[imgSize[1] - y-1,x] = [0,0,0]

    #code.interact(local=dict(globals(), **locals()))  


    

    rawimg = Image.fromarray(img, 'RGB')
    #rawimg.save('out.png')
    rawimg.save(sys.argv[1]+'.png')
    
if __name__=="__main__":
    main()
