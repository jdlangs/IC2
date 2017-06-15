import yaml

class TargetDefinition(yaml.YAMLObject):
  yaml_tag = u'!CalibrationTarget'
  def __init__(self, name, t_type, rows, cols, n_points, diameter, spacing, points):
    self.target_name = name
    self.target_type = t_type
    self.target_rows = rows
    self.target_cols = cols
    self.target_points = n_points
    self.circle_diameter = diameter
    self.spacing = spacing
    self.points = points

  def __repr__(self):
    return "%s(target_name=%r, target_type=%r, target_rows=%r, target_cols=%r, target_points=%r, circle_diameter=%r, spacing=%r, points=%r)" % (self.__class__.__name__, self.target_name, self.target_type, self.target_rows, self.target_cols, self.target_points, self.circle_diameter, self.spacing, self.points)

  def get_dict(self):
    return {'target_name':self.target_name, 'target_type':self.target_type, 'target_rows':self.target_rows, 'target_cols':self.target_cols, 'target_points':self.target_points, 'circle_diameter':self.circle_diameter, 'spacing':self.spacing, 'points':self.points}

# https://sourceforge.net/p/yaml/mailman/message/23596068/
def represent_dict(self, data):
  def key_function((key, value)):
    prio = {'target_name':0, 'target_type':1, 'target_rows':2, 'target_cols':3, 'target_points':4, 'circle_diameter':5, 'spacing':6, 'points':7}.get(key, 99)
    return (prio, key)
  items = data.get_dict().items()
  items.sort(key=key_function)
  return self.represent_mapping(u'tag:yaml.org,2002:map', items)

def generate_yaml(name, t_type, rows, cols, n_points, diameter, spacing, units):
  diameter = float(diameter)
  spacing = float(spacing)
  
  # Convert all units to meters (from mm and in)
  # Units defaulted to mm so this may not be necessary
  if units == 'in':
    print("in")
    diameter = 0.0254*diameter
    spacing = 0.0254*spacing
  elif units == 'mm':
    diameter = 0.001*diameter
    spacing = 0.001*spacing
  elif units == 'm':
    diameter = diameter
    spacing = spacing
  else:
    print("Provided units must be in in/mm/m!")

  # Convert type string to number: Chessboard = 0; CircleGrid = 1; MCircleGrid = 2;
  if t_type == 'checkerboard':
    target_type = 0
  elif t_type == 'circles':
    target_type = 1
  elif t_type == 'mcircles':
    target_type = 2
  else:
    print("Provided target type is not circles/mcircles/checkerboard!")

  # Generate points (does not work for asymmetric)
  points = generate_points(rows, cols, n_points, spacing)

  # Create YAML Object and write to it
  file_name = name + '.yaml'
  stream = file(file_name, 'w')
  yaml.add_representer(TargetDefinition, represent_dict)
  print yaml.dump(TargetDefinition(name=name, t_type=t_type, rows=rows, cols=cols, n_points=n_points, diameter=diameter, spacing=spacing, points=points), stream)

def generate_points(rows, cols, n_points, spacing):
  points = []
  assert(rows*cols == n_points)

  for i in range(0, rows):
    y = i*spacing
    for j in range(0, cols):
      x = j*spacing
      point = [x, y, 0.0000]
      points.append(point)

  return points
