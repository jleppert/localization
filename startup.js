const pm2 = require('pm2'),
      path = require('path');

var develRoot = path.join(__dirname, '../');

var processes = {
  index: path.join(develRoot, 'scan_review', 'scan_review.sh'),
  gm6020_can_redis_driver: path.join(develRoot, 'localization', 'bin', 'gm6020_can_redis_driver'),
  survive_redis_driver: path.join(develRoot, 'localization', 'bin', 'survive_redis_driver'),
  mecanum_drive_controller: path.join(develRoot, 'localization', 'bin', 'mecanum_drive_controller'),
  radar_data_pipeline: path.join(develRoot, 'radar-mvp-test', 'process_scan', 'radar_data_pipeline.sh')
};

console.log(processes);

function main(log = console.log) {
  pm2.connect(function(err) {
    if (err) {
      console.error(err)
      process.exit(2)
    }

    log('Stopping processes...');

    pm2.stop(processes.index, () => {
      pm2.stop(processes.radar_data_pipeline, () => {
        pm2.stop(processes.gm6020_can_redis_driver, () => {
          pm2.stop(processes.survive_redis_driver, () => {
            pm2.stop(processes.mecanum_drive_controller, () => {
              doStartup();
            });
          });
        });
      });
    });


    function doStartup() {
      pm2.start({
        script: processes.index
      }, function(err, apps) {
        log(err, apps);

        if(err) {
          return log('Could not start scan review app');
        }

        startupControlProcesses();
      });

      function startupControlProcesses() {
        pm2.start({
          script: processes.gm6020_can_redis_driver
        }, function(err, apps) {
          log(err, apps);

          if(err) {
            return log('Could not start motor driver');
          }

          pm2.start({
            script: processes.survive_redis_driver
          }, function(err, apps) {
            log(err, apps);

            if(err) {
              return log('Could not start localization');
            }

            pm2.start({
              script: processes.mecanum_drive_controller
            }, function(err, apps) {
              log(err, apps);

              if(err) {
                return log('Could not start drive controller');
              }

              pm2.start({
                script: processes.radar_data_pipeline
              }, function(err, apps) {
                log(err, apps);

                if(err) {
                  return log('Could not start radar data pipeline');
                }
              });
            });
          });
        });
      }
    }
  });
}

main();
