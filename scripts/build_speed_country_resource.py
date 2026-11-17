#!/usr/bin/env python3
"""Build release country resources from locally acquired, pinned GeoJSON sources.

No network or model operations. Input JSON: {sources:[{path,sha256,code,license,
attribution,source_url,version}], policy:{...}}. Output goes to an explicit staging
directory; never defaults to the running checkout. Every original source is
preserved by digest and attribution in provenance.json.
"""
import argparse
import gzip
import hashlib
import json
from pathlib import Path


def build(spec, output):
  countries=[]
  for source in spec['sources']:
    raw=Path(source['path']).read_bytes()
    if hashlib.sha256(raw).hexdigest()!=source['sha256']:
      raise ValueError('source digest mismatch')
    if not all(source.get(k) for k in ('license','attribution','source_url','version')):
      raise ValueError('source provenance incomplete')
    features=json.loads(raw)['features']
    polygons=[]
    for feature in features:
      geometry=feature['geometry']
      if geometry['type'] not in ('Polygon','MultiPolygon'):
        raise ValueError('unsupported geometry')
      polygons.extend(geometry['coordinates'] if geometry['type']=='MultiPolygon' else [geometry['coordinates']])
    points=[p for polygon in polygons for ring in polygon for p in ring]
    countries.append({'code':source['code'],'polygons':polygons,
      'bbox':[min(p[0] for p in points),min(p[1] for p in points),max(p[0] for p in points),max(p[1] for p in points)]})
  if not countries or len({c['code'] for c in countries})!=len(countries):
    raise ValueError('empty or duplicate countries')
  asset=gzip.compress(json.dumps({'countries':countries},separators=(',',':')).encode(),mtime=0)
  output=Path(output);output.mkdir(parents=True,exist_ok=False)
  (output/'borders.json.gz').write_bytes(asset)
  (output/'country-context.json').write_text(json.dumps({'asset':'borders.json.gz',
    'sha256':hashlib.sha256(asset).hexdigest(),'policy':spec['policy']},indent=2)+'\n')
  (output/'provenance.json').write_text(json.dumps(spec['sources'],indent=2)+'\n')


if __name__=='__main__':
  parser=argparse.ArgumentParser(description=__doc__)
  parser.add_argument('spec',type=Path)
  parser.add_argument('--output',type=Path,required=True)
  args=parser.parse_args()
  build(json.loads(args.spec.read_text()),args.output)
