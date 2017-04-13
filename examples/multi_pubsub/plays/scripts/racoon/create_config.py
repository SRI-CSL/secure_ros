#!/usr/bin/env python 

import os
import sys
import shutil 
import subprocess
import argparse
import yaml

def make_racoon_header( file_obj = sys.stdout ):
  file_obj.write( '# Racoon IKE daemon configuration file.\n#\n' )
  file_obj.write( 'log notify;\n' )
  file_obj.write( 'path certificate "/etc/racoon/certs";\n\n' )



def make_racoon_remote( file_obj, host_ip, host_name, peer_ip, peer_name ):
  file_obj.write( 'remote %s\n{\n' % peer_ip )
  file_obj.write( '  exchange_mode main;\n' )
  file_obj.write( '  lifetime time 12 hour;\n' )
  file_obj.write( '  certificate_type plain_rsa "%s";\n' % host_name )
  file_obj.write( '  peers_certfile plain_rsa "%s.pub";\n' % peer_name )
  file_obj.write( '  verify_cert off;\n' )
  file_obj.write( '  proposal {\n' )
  file_obj.write( '    encryption_algorithm 3des;\n' )
  file_obj.write( '    hash_algorithm sha256;\n' )
  file_obj.write( '    authentication_method rsasig;\n' )
  file_obj.write( '    dh_group modp1024;\n' )
  file_obj.write( '  }\n' )
  file_obj.write( '  generate_policy off;\n' )
  file_obj.write( '}\n\n' )



def make_racoon_sainfo( file_obj ):
  file_obj.write( 'sainfo anonymous\n' )
  file_obj.write( '{\n' )
  file_obj.write( '	pfs_group modp1024;\n' )
  file_obj.write( '	encryption_algorithm 3des;\n' )
  file_obj.write( '	authentication_algorithm hmac_sha256;\n' )
  file_obj.write( '	compression_algorithm deflate;\n' )
  file_obj.write( '}\n' )



def make_setkey_header( file_obj ):
  file_obj.write( '#!/usr/sbin/setkey -f\n#\n\n' )
  file_obj.write( 'flush;\n' )
  file_obj.write( 'spdflush;\n\n' )



def make_setkey_spd( file_obj, host_ip, peer_ip, val ):
  file_obj.write( 'spdadd %s %s any -P %s ipsec\n' % ( host_ip, peer_ip, val ) )
  file_obj.write( '\tesp/transport//require\n' )
  file_obj.write( '\tah/transport//require;\n\n' )



def make_racoon_conf( filename, host, ip_address ):
  """ Make racoon conf file 
      Note that the key is expected to be named as {hostname} and {hostname}.pub
      filename: configuration filename 
      host: hostname 
      ip_address: dict{ hostname: ip_address } 
  """
  if not os.path.exists( os.path.dirname( filename ) ): 
    os.makedirs( os.path.dirname( filename ) )
  peers = set( ip_address.keys() ) - set( [host] )
  with open( filename, "w" ) as f:
    make_racoon_header( f )
    for peer in peers:
      make_racoon_remote( f, ip_address[host], host, ip_address[peer], peer )
    make_racoon_sainfo( f )



def make_setkey_conf( filename, host, ip_address ):
  """ Make setkey conf file 
      Note that the key is expected to be named as {hostname} and {hostname}.pub
      filename: configuration filename 
      host: hostname 
      ip_address: dict{ hostname: ip_address } 
  """
  if not os.path.exists( os.path.dirname( filename ) ): 
    os.makedirs( os.path.dirname( filename ) )
  peers = set( ip_address.keys() ) - set( [host] )
  with open( filename, "w" ) as f:
    make_setkey_header( f )
    for peer in peers:
      make_setkey_spd( f, ip_address[host], ip_address[peer], 'out' )
      make_setkey_spd( f, ip_address[peer], ip_address[host], 'in' )



def make_config( doc, hostname, prefix ):
  racoon_conf = '%s/etc/racoon/racoon.conf' % prefix
  print( 'Writing to %s' % racoon_conf )
  make_racoon_conf( racoon_conf, hostname, doc )

  setkey_conf = '%s/etc/ipsec-tools.conf' % prefix
  print( 'Writing to %s' % setkey_conf )
  make_setkey_conf( setkey_conf, hostname, doc )



if __name__ == "__main__":
  parser = argparse.ArgumentParser( description = "Tool to create IPSec configuration files" )
  parser.add_argument( "hostname", metavar = "HOSTNAME", help = "hostname" )
  parser.add_argument( "security", metavar = "SECURITY_FILE", help = "YAML Security configuration" )
  parser.add_argument( "--prefix", metavar = "OUTPUT_PREFIX", default="", help = "Output folder prefix" )
  args = parser.parse_args()

  with open( args.security ) as istream:
    doc = yaml.load( istream )

  # ip_addresses is a dict of {hostname: ip_address}
  ip_addresses = doc["machines"]
  print( "ip_addresses: %s" % ip_addresses )
  make_config( ip_addresses, args.hostname, args.prefix )

