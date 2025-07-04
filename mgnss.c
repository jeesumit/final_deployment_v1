#include <mongoc/mongoc.h>
#include <bson.h>
#include <stdlib.h>
#include <stdio.h>

int main (int argc, char *argv[])
{  bson_json_reader_t *read;
   bson_error_t error;
   const char *filename;
   bson_t doc = BSON_INITIALIZER;
   long int a=0;
   filename = "data/gns.json";
   bson_t *b;
   
    mongoc_collection_t *collection;
    mongoc_client_t *client = NULL;
    mongoc_database_t *database = NULL;
    const char *dbname;
   
    // Initialize the MongoDB C Driver.
    mongoc_init();
    client = mongoc_client_new("mongodb+srv://chatterjeesumit889:robot@clustera.rysk1qu.mongodb.net/?retryWrites=true&w=majority&appName=ClusterA");
    if (!client) {
        printf("---client---\n");
        fprintf(stderr, "Failed to create a MongoDB client.\n");
    }
    database = mongoc_client_get_database (client, "tractor6065");
    if(!database){
       printf("fail");
       }
    dbname = mongoc_database_get_name (database);
    collection = mongoc_client_get_collection (client,dbname, "gnss");
    if(!collection){
      printf("fail-cc");
}

while(1){
    
  if (!(read = bson_json_reader_new_from_file (filename, &error))) {
            fprintf (stderr, "Failed to open \"%s\": %s %ld \n ", filename, error.message,a);
            
         }
        
  if(!(bson_json_reader_read (read, &doc, &error))){
  fprintf (stderr, "READ ERROR \"%s\": %s\n", filename, error.message);
        }
    bson_json_reader_destroy (read);
   if (!mongoc_collection_insert_one (collection, &doc, NULL, NULL, &error)) {
    fprintf (stderr, "Insert one operation failed: %s\n", error.message);
         } 
    
    bson_reinit (&doc);
    a+=1;
    printf("------Done:%ld \n",a);
    
    }
    
    bson_destroy (&doc);
    
    mongoc_database_destroy(database);
    mongoc_client_destroy(client);
    mongoc_cleanup();
   return 0;
}





