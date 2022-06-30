const pm2 = require('pm2');

pm2.connect(function(err) {
  if (err) {
    console.error(err)
    process.exit(2)
  }

  pm2.stop('index', () => {
    pm2.stop('gm6020_can_redis_driver', () => {
      pm2.stop('survive_redis_driver', () => {
        pm2.stop('mecanum_driver_controller', () => {
          doStartup();
        });
      });
    });
  });


  function doStartup() {
    pm2.start({
      name: 'index'
    }, function(err, apps) {
      console.log(err, apps);

      if(err) {
        console.log('Could not start scan review app');
        pm2.disconnect();
        process.exit(1);
      }

      startupControlProcesses();
    });

    function startupControlProcesses() {
      pm2.start({
        name: 'gm6020_can_redis_driver'
      }, function(err, apps) {
        console.log(err, apps);

        if(err) {
          console.log('Could not start motor driver');
          pm2.disconnect();
          process.exit(1);
        }

        pm2.start({
          name: 'survive_redis_driver'
        }, function(err, apps) {
          console.log(err, apps);

          if(err) {
            console.log('Could not start localization');
            pm2.disconnect();
            process.exit(1);
          }

          pm2.start({
            name: 'mecanum_drive_controller'
          }, function(err, apps) {
            console.log(err, apps);

            pm2.disconnect();
            process.exit(0);

            if(err) {
              console.log('Could not start drive controller');
              pm2.disconnect();
              process.exit(1);
            }
          });
        });
      });
    }
  }
});