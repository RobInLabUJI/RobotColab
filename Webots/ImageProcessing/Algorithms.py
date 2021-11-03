import numpy as np
import IPython.display
import PIL.Image

def display(image, size=(300,200)):
  """Display the image in the notebook."""
  if image.dtype == np.dtype('uint8'):
    PILimage = PIL.Image.fromarray(image)
  elif image.dtype == np.dtype('float64'):
    intimage = np.round(image/np.max(image)*255).astype('uint8')
    PILimage = PIL.Image.fromarray(intimage)
  IPython.display.display(PILimage.resize(size))

