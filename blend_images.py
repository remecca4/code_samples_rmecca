'''
Uses Gaussian and Laplacian pyramids to blend two images
'''
import numpy as np, matplotlib as mpl, matplotlib.pyplot as plt, os
import imageio # For loading images
from abc import update_abstractmethods
from PIL import Image
import scipy
def pyramid_upsample(img, kernel_size=(5,5)):
  """
  Upsamples the given pyramid image.
  Input:
    - img: an image of shape M x N x C
    - kernel_size: a tuple representing the shape of the 2D kernel
  Returns:
    - upsampled: an image represented as an array of shape 2M x 2N x C
  """
  M=img.shape[0]
  N=img.shape[1]
  C=img.shape[2]
  upsampled = None

  upsampled = np.zeros((2*M,2*N,C))
  upsampled[::2, ::2,:] = img
  for i in range(0,C):
    filter = scipy.ndimage.gaussian_filter(upsampled[:, :, i], sigma=1, radius=kernel_size)
    upsampled[:, :, i] = filter * 4
  return upsampled

def pyramid_downsample(img, kernel_size=(5,5)):
  """
  Downsamples the given pyramid image.
  Input:
    - img: an image of shape M x N x C
    - kernel_size: a tuple representing the shape of the 2D kernel
  Returns:
    - downsampled: an image of shape M/2 x N/2 x C
  """
  M=img.shape[0]
  N=img.shape[1]
  C=img.shape[2]
  downsampled = np.zeros((M//2,N//2,C))
  for i in range(0,C):
   filter= scipy.ndimage.gaussian_filter(img[:, :, i], sigma=1, radius=kernel_size)
   downsampled[:, :, i] = filter[::2, ::2][:downsampled.shape[0], :downsampled.shape[1]]
  return downsampled

def gen_gaussian_pyramid(img, num_levels):
  """
  Generates an entire Gaussian pyramid.
  Input:
    - img: an image of shape M x N x C
    - num_levels: number of levels in the Gaussian pyramid
  Return:
    - gp: list, the generated levels (imgs) of the Gaussian pyramid
  """
  gp = [img]
  for i in range(1,num_levels) :
    img=pyramid_downsample(img)
    gp.append(img)
  return gp

def gen_laplacian_pyramid(gp, num_levels):
  """
  Generates an entire Laplacian pyramid.
  Input:
    gp: list, levels of a Gaussian pyramid
  Return:
    lp: list, the generated levels (imgs) of the Laplacian pyramid
  """
  lp = []
  lp.append(gp[num_levels-1])
  for i in range(num_levels-2,-1,-1) :
    upsampled_img = pyramid_upsample(gp[i+1])
    img=gp[i]-upsampled_img
    lp.append(img)
  return lp

def reconstruct_img(lp):
  """
  Reconstructs an image using a laplacian pyramid.
  Input:
    lp: list, levels (imgs) of a Laplacian pyramid
  Return:
    recon_img: reconstructed image
  """
  recon_img = lp[0]
  for i in range(1,len(lp)):
   upsampled_img=pyramid_upsample(recon_img)
   recon_img=upsampled_img+lp[i]
  return recon_img

def pyramid_blend(img1, img2, mask, num_levels=6):
  
  """
  This function produces the Laplacian pyramid blend of two images.
  Input:
    - img1: N x M x C uint8 array image
    - img2: N x M x C uint8 array image
    - mask: N x M array, all elements are either 0s or 1s
    - num_levels: int, height of the pyramid
  Return:
    - img_blend: the blended image, an N x M x C uint8 array
  """

  gp1, gp2, gpm = None, None, None
  lp1, lp2, lpm = None, None, None
  img_blend = None
  gp1=gen_gaussian_pyramid(img1,num_levels)
  gp2=gen_gaussian_pyramid(img2,num_levels)
  gpm=gen_gaussian_pyramid(mask,num_levels)
  lp1=gen_laplacian_pyramid(gp1,num_levels)
  lp2=gen_laplacian_pyramid(gp2,num_levels)
  lpm=gen_laplacian_pyramid(gpm,num_levels)
  lp_blend=[]
  for i in range(0,num_levels):
    lp_blend.append(lp1[i]*gpm[num_levels-i-1]+lp2[i]*(1-gpm[num_levels-i-1]))
  img_blend=reconstruct_img(lp_blend)
  return img_blend

if __name__=='__main__':
 # example usage
 img_paths = ['images/platypus.webp', 'images/space.jpg']
 img1, img2 = None, None
 img1 = Image.open(img_paths[0])
 img1 = np.array(img1.resize((512, 400)).convert("RGB"))
 img2 = Image.open(img_paths[1])
 img2 = np.array(img2.resize((512, 400)).convert("RGB"))
 # create mask
 mask = np.zeros_like(img1)
 for i in range(0, mask.shape[0]):
  for j in range(0,mask.shape[1]):
    for c in range(0,mask.shape[2]):
      if(img1[i,j,0]>60):
       mask[i,j,c]=1
 for i in range(280, mask.shape[0]):
  for j in range(0,mask.shape[1]):
    for c in range(0,mask.shape[2]):
       mask[i,j,c]=0
 for i in range(0, mask.shape[0]-250):
  for j in range(0,mask.shape[1]):
    for c in range(0,mask.shape[2]):
       mask[i,j,c]=0


 fig, axs = plt.subplots(2, 5, figsize=(20, 10))
 fig.suptitle('Pyramid Blending Results', fontsize=16)

 # Plotting image 1 - Platypus
 axs[0, 0].imshow(img1)
 axs[0, 0].set_title('Platypus')
 axs[0, 0].axis('off')

 # Plotting image 2 - Space
 axs[0, 1].imshow(img2)
 axs[0, 1].set_title('Space')
 axs[0, 1].axis('off')

 # Plotting the mask
 axs[0, 2].imshow(mask * 255)
 axs[0, 2].set_title('Mask')
 axs[0, 2].axis('off')

 # Placeholder for layout consistency
 axs[0, 3].axis('off')
 axs[0, 4].axis('off')

 for idx, n_l in enumerate(range(1, 6)):
    img_blend = pyramid_blend(img1.astype(float), img2.astype(float), mask.astype(float), num_levels=n_l)
    img_blend = img_blend.astype(np.uint8)
    row, col = divmod(idx, 5)
    axs[1, col].imshow(img_blend)
    axs[1, col].set_title(f'Levels = {n_l}')
    axs[1, col].axis('off')

 plt.tight_layout()
 plt.show()
